// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Super MMU TLB using super_tlb_entry_t:
// - Variable page size: 256B, 4KB, 64KB, 1MB
// - Tag = {lin[7:0], selected_exp[3:0]}, page_sz encodes which exp level
// - phys_page stores full leaf PTE[31:8] directly

module ibex_super_tlb import ibex_pkg::*; #(
  parameter int unsigned TLB_ENTRIES = 4
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Translation Request Interface
  input  logic                  req_i,
  input  logic [31:0]           vaddr_i,
  input  priv_lvl_e             priv_lvl_i,
  input  logic                  mstatus_sum_i,
  input  logic                  mstatus_mxr_i,
  input  logic                  is_instruction_i,
  input  logic                  is_store_i,

  // Translation Response Interface
  output logic [31:0]           paddr_o,
  output logic                  hit_o,
  output logic                  page_fault_o,

  // Control and Invalidation
  input  logic                  flush_i,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [31:0]           csr_satp_i,
  /* verilator lint_on UNUSEDSIGNAL */

  // PTW Fill Interface
  input  logic                  ptw_write_i,
  input  super_tlb_entry_t      ptw_entry_i
);

  // Page size encoding
  localparam logic [1:0] PG_256B  = 2'b00;
  localparam logic [1:0] PG_4KB   = 2'b01;
  localparam logic [1:0] PG_64KB  = 2'b10;
  localparam logic [1:0] PG_1MB   = 2'b11;

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

  // Registers
  super_tlb_entry_t [TLB_ENTRIES-1:0]    tlb_mem_q, tlb_mem_d;
  logic [$clog2(TLB_ENTRIES)-1:0]        replace_idx_q, replace_idx_d;

  // Combinational signals
  logic [1:0]            cur_pg_code;
  logic [11:0]           cur_tag12;
  logic                  match_found;
  /* verilator lint_off UNUSEDSIGNAL */
  super_tlb_entry_t      matched_entry;
  /* verilator lint_on UNUSEDSIGNAL */
  logic                  mmu_enabled;
  logic                  perm_fault;

  // VA decode
  assign cur_pg_code = super_pg_code(vaddr_i);
  assign cur_tag12   = super_tag12(vaddr_i, cur_pg_code);

  // MMU active when satp.MODE=1 and not M-mode
  assign mmu_enabled = csr_satp_i[31] && (priv_lvl_i != PRIV_LVL_M);

  // Fully associative tag matching
  always_comb begin
    match_found   = 1'b0;
    matched_entry = '0;

    if (mmu_enabled && req_i) begin
      for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
        if (tlb_mem_q[i].v &&
            (tlb_mem_q[i].page_sz == cur_pg_code) &&
            (tlb_mem_q[i].tag     == cur_tag12)) begin
          match_found   = 1'b1;
          matched_entry = tlb_mem_q[i];
        end
      end
    end
  end

  // Permission Evaluation
  always_comb begin
    perm_fault = 1'b0;

    if (match_found) begin
      // User/supervisor accessibility
      if (priv_lvl_i == PRIV_LVL_U && !matched_entry.u) begin
        perm_fault = 1'b1;
      end else if (priv_lvl_i == PRIV_LVL_S &&
                   matched_entry.u &&
                   (!mstatus_sum_i || is_instruction_i)) begin
        perm_fault = 1'b1;
      end

      // Access type checking
      if (is_instruction_i && !matched_entry.x) begin
        perm_fault = 1'b1;
      end else if (!is_instruction_i && !is_store_i) begin
        if (!matched_entry.r && !(mstatus_mxr_i && matched_entry.x)) begin
          perm_fault = 1'b1;
        end
      end else if (!is_instruction_i && is_store_i && !matched_entry.w) begin
        perm_fault = 1'b1;
      end

      // Accessed/Dirty enforcement
      if (!matched_entry.a) begin
        perm_fault = 1'b1;
      end else if (is_store_i && !matched_entry.d) begin
        perm_fault = 1'b1;
      end
    end
  end

  // TLB Memory Update
  always_comb begin
    tlb_mem_d     = tlb_mem_q;
    replace_idx_d = replace_idx_q;

    if (flush_i) begin
      for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
        tlb_mem_d[i].v = 1'b0;
      end
    end else if (ptw_write_i) begin
      tlb_mem_d[replace_idx_q] = ptw_entry_i;
      replace_idx_d            = replace_idx_q + 1'b1;
    end
  end

  // Flip-Flops
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tlb_mem_q     <= '0;
      replace_idx_q <= '0;
    end else begin
      tlb_mem_q     <= tlb_mem_d;
      replace_idx_q <= replace_idx_d;
    end
  end

  // Output
  assign hit_o        = req_i && (match_found || !mmu_enabled);
  assign page_fault_o = req_i && mmu_enabled && match_found && perm_fault;

  // Physical address formation — straightforward with full phys_page
  always_comb begin
    paddr_o = vaddr_i;

    if (mmu_enabled && match_found) begin
      unique case (matched_entry.page_sz)
        PG_1MB:  paddr_o = {matched_entry.phys_page[23:12], vaddr_i[19:0]};
        PG_64KB: paddr_o = {matched_entry.phys_page[23:8],  vaddr_i[15:0]};
        PG_4KB:  paddr_o = {matched_entry.phys_page[23:4],  vaddr_i[11:0]};
        default: paddr_o = {matched_entry.phys_page,         vaddr_i[7:0]};
      endcase
    end
  end

endmodule
