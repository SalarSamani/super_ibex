// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module ibex_tlb import ibex_pkg::*; #(
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
  input  logic [31:0]           csr_satp_i,
  
  // PTW Fill Interface
  input  logic                  ptw_write_i,
  input  tlb_entry_t            ptw_entry_i
);
  
  // Registers (Flip-Flops) and Next-State signals
  tlb_entry_t [TLB_ENTRIES-1:0]         tlb_mem_q;
  tlb_entry_t [TLB_ENTRIES-1:0]         tlb_mem_d;
  logic       [$clog2(TLB_ENTRIES)-1:0] replace_idx_q;
  logic       [$clog2(TLB_ENTRIES)-1:0] replace_idx_d;
  
  // Combinational signals
  logic [19:0]                    current_vpn;
  logic                           match_found;
  tlb_entry_t                     matched_entry;
  logic                           mmu_enabled;
  logic                           perm_fault;

  // Address & Tag Matching
  
  assign current_vpn = vaddr_i[31:12];
  
  // Translation is active only if csr_satp_i.MODE (bit 31) is 1
  assign mmu_enabled = csr_satp_i[31];

  // Fully associative tag matching
  always_comb begin
    match_found   = 1'b0;
    matched_entry = '0;
    
    if (mmu_enabled) begin
      for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
        if (tlb_mem_q[i].v && (tlb_mem_q[i].vpn == current_vpn)) begin
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
      // 1. User-mode accessibility check
      if (priv_lvl_i == PRIV_LVL_U && !matched_entry.u) begin
        perm_fault = 1'b1; // U-mode cannot access S-mode pages
      end else if (priv_lvl_i == PRIV_LVL_S && 
                   matched_entry.u && 
                   (!mstatus_sum_i || is_instruction_i)) begin
        // S-mode accessing U-mode pages requires SUM=1 
        // (Access is strictly forbidden for instruction fetches)
        perm_fault = 1'b1; 
      end
      
      // 2. Access Type Checking
      if (is_instruction_i && !matched_entry.x) begin
        perm_fault = 1'b1; // Fetching from non-executable page
      end else if (!is_instruction_i && !is_store_i) begin 
        // Load operation: Requires R=1, OR (MSTATUS_MXR=1 AND X=1)
        if (!matched_entry.r && !(mstatus_mxr_i && matched_entry.x)) begin
          perm_fault = 1'b1; 
        end
      end else if (!is_instruction_i && is_store_i && !matched_entry.w) begin
        perm_fault = 1'b1; // Storing to non-writable page
      end

      // 3. Software-Managed Accessed and Dirty Bit Enforcement
      if (!matched_entry.a) begin
        // If the hardware detects an access to a page with A=0, it must fault
        perm_fault = 1'b1;
      end else if (is_store_i && !matched_entry.d) begin
        // If the hardware detects a store to a page with D=0, it must fault
        perm_fault = 1'b1;
      end
    end
  end
  
  // TLB Memory Update Logic
  
  always_comb begin
    // Default assignments: hold current values
    tlb_mem_d     = tlb_mem_q;
    replace_idx_d = replace_idx_q;

    if (flush_i) begin
      // Invalidate all entries on SFENCE.VMA
      for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
        tlb_mem_d[i].v = 1'b0;
      end
    end else if (ptw_write_i) begin
      // Write the incoming entry from the PTW and increment the counter
      tlb_mem_d[replace_idx_q] = ptw_entry_i;
      replace_idx_d            = replace_idx_q + 1'b1;
    end
  end

  // Flip-Flops (Registers)
  
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tlb_mem_q     <= '0;
      replace_idx_q <= '0;
    end else begin
      tlb_mem_q     <= tlb_mem_d;
      replace_idx_q <= replace_idx_d;
    end
  end

  // Output Generation
  
  // A hit occurs if the MMU is disabled (passthrough) OR if the tag matches.
  assign hit_o        = req_i && (match_found || !mmu_enabled);
  
  // A page fault is explicitly asserted if permissions fail during an active translation
  assign page_fault_o = req_i && mmu_enabled && match_found && perm_fault;
  
  // Physical address concatenation: PPN from TLB, offset from VA
  assign paddr_o      = mmu_enabled ? {matched_entry.ppn, vaddr_i[11:0]} : vaddr_i;

endmodule