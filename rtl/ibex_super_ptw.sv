// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Super MMU PTW using super_tlb_entry_t:
// - Level 1 "root" is 16 CSR registers (l1pte_i[0..15]), indexed by lin1 (VA[31:28])
// - Level 2 is a 16-entry table in memory, indexed by lin0 (VA[27:24])
// - Level 3 is a 16-entry table in memory, indexed by exp_sel (exp3/2/1/0 per page-size rule)
// - Each PTE is 32 bits: [31:8] address/base field, [7:0] flags (V,R,W,X,U,G,A,D)

module ibex_super_ptw import ibex_pkg::*; (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // TLB Request Interface
  input  logic                  itlb_req_i,
  input  logic [31:0]           itlb_vaddr_i,
  input  logic                  dtlb_req_i,
  input  logic [31:0]           dtlb_vaddr_i,

  // L1 PTE Registers (root table in core)
  input  logic [31:0]           l1pte_i [16],

  // TLB Fill Interface
  output logic                  itlb_write_o,
  output logic                  dtlb_write_o,
  output super_tlb_entry_t      tlb_entry_o,
  output logic                  ptw_error_o,

  // External Memory Interface
  output logic                  ptw_mem_req_o,
  output logic [31:0]           ptw_mem_addr_o,
  input  logic                  ptw_mem_gnt_i,
  input  logic                  ptw_mem_rvalid_i,
  input  logic [31:0]           ptw_mem_rdata_i,
  input  logic                  ptw_mem_err_i,

  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [31:0]           satp_i
  /* verilator lint_on UNUSEDSIGNAL */
);

  // Page size encoding
  localparam logic [1:0] PG_256B  = 2'b00;
  localparam logic [1:0] PG_4KB   = 2'b01;
  localparam logic [1:0] PG_64KB  = 2'b10;
  localparam logic [1:0] PG_1MB   = 2'b11;

  // PTE bit positions
  localparam int PTE_V = 0;
  localparam int PTE_R = 1;
  localparam int PTE_W = 2;
  localparam int PTE_X = 3;
  localparam int PTE_U = 4;
  localparam int PTE_G = 5;
  localparam int PTE_A = 6;
  localparam int PTE_D = 7;

  // VA decoding helpers
  function automatic logic [1:0] super_pg_code(input logic [31:0] va);
    logic unused_va;
    unused_va = ^{va[31:24], va[11:0]};
    if (va[23:20] != 4'h0) begin
      return PG_1MB;
    end else if (va[19:16] != 4'h0) begin
      return PG_64KB;
    end else if (va[15:12] != 4'h0) begin
      return PG_4KB;
    end else begin
      return PG_256B;
    end
  endfunction

  function automatic logic [3:0] super_exp_sel(input logic [31:0] va, input logic [1:0] pg_code);
    logic unused_va;
    unused_va = ^{va[31:24], va[7:0]};
    unique case (pg_code)
      PG_1MB:  return va[23:20];
      PG_64KB: return va[19:16];
      PG_4KB:  return va[15:12];
      default: return va[11:8];
    endcase
  endfunction

  function automatic logic [11:0] super_tag12(input logic [31:0] va, input logic [1:0] pg_code);
    logic unused_va;
    unused_va = ^{va[7:0]};
    return {va[31:24], super_exp_sel(va, pg_code)};
  endfunction

  // PTE validation helpers
  function automatic logic pte_has_reserved_rw(input logic [31:0] pte);
    return (!pte[PTE_R] && pte[PTE_W]);
  endfunction

  function automatic logic pte_is_bad_pointer(input logic [31:0] pte);
    return (!pte[PTE_V]) ||
           pte_has_reserved_rw(pte) ||
           pte[PTE_R] || pte[PTE_W] || pte[PTE_X];
  endfunction

  function automatic logic pte_is_bad_leaf(input logic [31:0] pte);
    return (!pte[PTE_V]) ||
           pte_has_reserved_rw(pte) ||
           (!pte[PTE_R] && !pte[PTE_X]);
  endfunction

  function automatic logic [31:0] pte_base_addr(input logic [31:0] pte);
    logic unused_pte;
    unused_pte = ^{pte[7:0]};
    return {pte[31:8], 8'h00};
  endfunction

  // FSM states
  typedef enum logic [2:0] {
    IDLE,
    L2_REQ,
    L2_WAIT,
    L3_REQ,
    L3_WAIT
  } ptw_state_e;

  // Registers
  ptw_state_e   state_q, state_d;
  logic         servicing_itlb_q, servicing_itlb_d;
  logic [31:0]  active_vaddr_q, active_vaddr_d;
  logic [31:0]  l3_base_addr_q, l3_base_addr_d;

  // Decoded fields from active_vaddr_q
  logic [1:0]  pg_code;
  logic [3:0]  exp_sel;
  logic [3:0]  lin1;
  logic [3:0]  lin0;

  assign pg_code = super_pg_code(active_vaddr_q);
  assign exp_sel = super_exp_sel(active_vaddr_q, pg_code);
  assign lin1    = active_vaddr_q[31:28];
  assign lin0    = active_vaddr_q[27:24];

  // FSM
  always_comb begin
    state_d          = state_q;
    servicing_itlb_d = servicing_itlb_q;
    active_vaddr_d   = active_vaddr_q;
    l3_base_addr_d   = l3_base_addr_q;

    ptw_mem_req_o    = 1'b0;
    ptw_mem_addr_o   = '0;
    itlb_write_o     = 1'b0;
    dtlb_write_o     = 1'b0;
    ptw_error_o      = 1'b0;
    tlb_entry_o      = '0;

    unique case (state_q)

      IDLE: begin
        if (dtlb_req_i || itlb_req_i) begin
          servicing_itlb_d = itlb_req_i && !dtlb_req_i;
          active_vaddr_d   = (itlb_req_i && !dtlb_req_i) ? itlb_vaddr_i : dtlb_vaddr_i;
          state_d          = L2_REQ;
        end
      end

      L2_REQ: begin
        logic [31:0] l1pte;
        l1pte = l1pte_i[lin1];

        if (pte_is_bad_pointer(l1pte)) begin
          ptw_error_o = 1'b1;
          state_d     = IDLE;
        end else begin
          logic [31:0] l1_base;
          logic [31:0] l2_addr;

          l1_base = pte_base_addr(l1pte);
          l2_addr = l1_base + {26'b0, lin0, 2'b00};

          ptw_mem_req_o  = 1'b1;
          ptw_mem_addr_o = l2_addr;

          if (ptw_mem_gnt_i) begin
            state_d = L2_WAIT;
          end
        end
      end

      L2_WAIT: begin
        if (ptw_mem_rvalid_i) begin
          logic [31:0] l2pte;
          l2pte = ptw_mem_rdata_i;

          if (ptw_mem_err_i || pte_is_bad_pointer(l2pte)) begin
            ptw_error_o = 1'b1;
            state_d     = IDLE;
          end else begin
            l3_base_addr_d = pte_base_addr(l2pte);
            state_d        = L3_REQ;
          end
        end
      end

      L3_REQ: begin
        logic [31:0] l3_addr;
        l3_addr = l3_base_addr_q + {26'b0, exp_sel, 2'b00};

        ptw_mem_req_o  = 1'b1;
        ptw_mem_addr_o = l3_addr;

        if (ptw_mem_gnt_i) begin
          state_d = L3_WAIT;
        end
      end

      L3_WAIT: begin
        if (ptw_mem_rvalid_i) begin
          logic [31:0] leaf_pte;
          logic [23:0] phys_page;

          leaf_pte  = ptw_mem_rdata_i;
          phys_page = leaf_pte[31:8];

          if (ptw_mem_err_i || pte_is_bad_leaf(leaf_pte)) begin
            ptw_error_o = 1'b1;
            state_d     = IDLE;
          end else begin
            // Alignment checks
            logic align_fault;
            align_fault = 1'b0;

            unique case (pg_code)
              PG_1MB:  align_fault = (phys_page[11:0] != 12'h000);
              PG_64KB: align_fault = (phys_page[7:0]  != 8'h00);
              PG_4KB:  align_fault = (phys_page[3:0]  != 4'h0);
              default: align_fault = 1'b0;
            endcase

            if (align_fault) begin
              ptw_error_o = 1'b1;
              state_d     = IDLE;
            end else begin
              // Build super_tlb_entry_t directly
              tlb_entry_o.tag       = super_tag12(active_vaddr_q, pg_code);
              tlb_entry_o.page_sz   = pg_code;
              tlb_entry_o.phys_page = phys_page;
              tlb_entry_o.v         = 1'b1;
              tlb_entry_o.r         = leaf_pte[PTE_R];
              tlb_entry_o.w         = leaf_pte[PTE_W];
              tlb_entry_o.x         = leaf_pte[PTE_X];
              tlb_entry_o.u         = leaf_pte[PTE_U];
              tlb_entry_o.g         = leaf_pte[PTE_G];
              tlb_entry_o.a         = leaf_pte[PTE_A];
              tlb_entry_o.d         = leaf_pte[PTE_D];

              if (servicing_itlb_q) begin
                itlb_write_o = 1'b1;
              end else begin
                dtlb_write_o = 1'b1;
              end

              state_d = IDLE;
            end
          end
        end
      end

      default: begin
        state_d = IDLE;
      end
    endcase
  end

  // Flip-Flops
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q          <= IDLE;
      servicing_itlb_q <= 1'b0;
      active_vaddr_q   <= '0;
      l3_base_addr_q   <= '0;
    end else begin
      state_q          <= state_d;
      servicing_itlb_q <= servicing_itlb_d;
      active_vaddr_q   <= active_vaddr_d;
      l3_base_addr_q   <= l3_base_addr_d;
    end
  end

endmodule
