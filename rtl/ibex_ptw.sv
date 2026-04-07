// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module ibex_ptw import ibex_pkg::*; (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  
  // TLB Request Interface
  input  logic                  itlb_req_i,
  input  logic [31:0]           itlb_vaddr_i,
  input  logic                  dtlb_req_i,
  input  logic [31:0]           dtlb_vaddr_i,
  
  // TLB Fill Interface
  output logic                  itlb_write_o,
  output logic                  dtlb_write_o,
  output tlb_entry_t            tlb_entry_o,
  output logic                  ptw_error_o,     // Triggers Page Fault on miss resolve
  
  // External Memory Interface (Multiplexed into LSU)
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

  // Types and Signals
  
  typedef enum logic [2:0] {
    IDLE,
    LVL1_REQ, 
    LVL1_WAIT,
    LVL0_REQ, 
    LVL0_WAIT
  } ptw_state_e;
  
  // Registers (Flip-Flops) and Next-State signals
  ptw_state_e   state_q;
  ptw_state_e   state_d;
  logic         servicing_itlb_q;
  logic         servicing_itlb_d;
  logic [31:0]  active_vaddr_q;
  logic [31:0]  active_vaddr_d;
  logic [21:0]  current_ppn_q;
  logic [21:0]  current_ppn_d;
  
  // Combinational signals
  /* verilator lint_off UNUSEDSIGNAL */
  sv32_pte_t   incoming_pte;
  /* verilator lint_on UNUSEDSIGNAL */

  // Input Parsing

  assign incoming_pte = ptw_mem_rdata_i;

  // FSM and Walk Logic

  always_comb begin
    // Default state assignments
    state_d          = state_q;
    servicing_itlb_d = servicing_itlb_q;
    active_vaddr_d   = active_vaddr_q;
    current_ppn_d    = current_ppn_q;
    
    // Default output assignments
    ptw_mem_req_o  = 1'b0;
    ptw_mem_addr_o = '0;
    itlb_write_o   = 1'b0;
    dtlb_write_o   = 1'b0;
    ptw_error_o    = 1'b0;
    tlb_entry_o    = '0;
    
    unique case (state_q)
      IDLE: begin
        // Arbitration: DTLB requests take strict priority over ITLB to prevent deadlocks
        if (dtlb_req_i || itlb_req_i) begin
          servicing_itlb_d = itlb_req_i && !dtlb_req_i; 
          active_vaddr_d   = (itlb_req_i && !dtlb_req_i) ? itlb_vaddr_i : dtlb_vaddr_i;
          current_ppn_d    = satp_i[21:0]; // Initialize with satp.PPN
          state_d          = LVL1_REQ;
        end
      end
      
      LVL1_REQ: begin
        ptw_mem_req_o  = 1'b1;
        // Level 1 Address Calculation: Base_PPN * 4096 + VPN[1] * 4
        ptw_mem_addr_o = {current_ppn_q[19:0], active_vaddr_q[31:22], 2'b00};
        
        if (ptw_mem_gnt_i) begin
          state_d = LVL1_WAIT;
        end
      end
      
      LVL1_WAIT: begin
        if (ptw_mem_rvalid_i) begin
          // Verify basic validity and catch reserved RW encodings
          if (ptw_mem_err_i || !incoming_pte.v || (!incoming_pte.r && incoming_pte.w)) begin
            ptw_error_o = 1'b1; 
            state_d     = IDLE;
            
          end else if (incoming_pte.r || incoming_pte.x) begin
            // Leaf detected at Level 1: 4 MiB Superpage
            if (incoming_pte.ppn0 != 0) begin
              ptw_error_o = 1'b1; // Misaligned superpage fault
            end else begin
              tlb_entry_o.vpn = active_vaddr_q[31:12];
              // Superpage PA mapping forces lower PPN bits to match lower VPN bits
              tlb_entry_o.ppn = {incoming_pte.ppn1, active_vaddr_q[21:12]};
              tlb_entry_o.u   = incoming_pte.u; 
              tlb_entry_o.x   = incoming_pte.x;
              tlb_entry_o.w   = incoming_pte.w; 
              tlb_entry_o.r   = incoming_pte.r;              tlb_entry_o.a   = incoming_pte.a;
              tlb_entry_o.d   = incoming_pte.d;              tlb_entry_o.v   = 1'b1;
              
              if (servicing_itlb_q) begin
                itlb_write_o = 1'b1;
              end else begin
                dtlb_write_o = 1'b1;
              end
            end
            state_d = IDLE;
            
          end else begin
            // Pointer detected. Traverse to Level 0.
            current_ppn_d = {incoming_pte.ppn1, incoming_pte.ppn0};
            state_d       = LVL0_REQ;
          end
        end
      end
      
      LVL0_REQ: begin
        ptw_mem_req_o  = 1'b1;
        // Level 0 Address Calculation: Next_PPN * 4096 + VPN[0] * 4
        ptw_mem_addr_o = {current_ppn_q[19:0], active_vaddr_q[21:12], 2'b00};
        
        if (ptw_mem_gnt_i) begin
          state_d = LVL0_WAIT;
        end
      end
      
      LVL0_WAIT: begin
        if (ptw_mem_rvalid_i) begin
          // Verify validity and ensure it is a leaf (Level 0 cannot be a pointer)
          if (ptw_mem_err_i || !incoming_pte.v || 
             (!incoming_pte.r && incoming_pte.w) || 
             (!incoming_pte.r && !incoming_pte.x)) begin
            
            ptw_error_o = 1'b1;
          end else begin
            tlb_entry_o.vpn = active_vaddr_q[31:12];
            tlb_entry_o.ppn = {incoming_pte.ppn1, incoming_pte.ppn0};
            tlb_entry_o.u   = incoming_pte.u; 
            tlb_entry_o.x   = incoming_pte.x;
            tlb_entry_o.w   = incoming_pte.w; 
            tlb_entry_o.r   = incoming_pte.r;            
            tlb_entry_o.a   = incoming_pte.a;
            tlb_entry_o.d   = incoming_pte.d;            
            tlb_entry_o.v   = 1'b1;
            
            if (servicing_itlb_q) begin
              itlb_write_o = 1'b1;
            end else begin
              dtlb_write_o = 1'b1;
            end
          end
          state_d = IDLE;
        end
      end
      
      default: state_d = IDLE;
    endcase
  end

  // Flip-Flops (Registers)

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q          <= IDLE;
      servicing_itlb_q <= 1'b0;
      active_vaddr_q   <= '0;
      current_ppn_q    <= '0;
    end else begin
      state_q          <= state_d;
      servicing_itlb_q <= servicing_itlb_d;
      active_vaddr_q   <= active_vaddr_d;
      current_ppn_q    <= current_ppn_d;
    end
  end

endmodule
