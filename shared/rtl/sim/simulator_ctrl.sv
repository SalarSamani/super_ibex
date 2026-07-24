// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Module for communicating with the simulator that interfaces via the memory
 * system.
 *
 * Contains two registers
 *
 * * 0x0 - CHAR_OUT_ADDR - [7:0] of write data output via output_char DPI call
 * and SimOutputManager (see dv/common/cpp/sim_output_manager.cc)
 *
 * * 0x8 - SIM_CTRL_ADDR - Write 1 to bit 0 to halt sim
 *
 * The slightly odd spacing is because we also use SIM_CTRL_ADDR when
 * simulating simple_system code with Spike, which requires the address to be
 * 64-bit aligned.
 *
 */

module simulator_ctrl #(
  // passed to simulator via log_name of output_char DPI call
  parameter string LogName    = "ibex_out.log",
  parameter string InputFile  = "stdin.txt",
  // If set flush on every char (useful for monitoring output whilst
  // simulation is running).
  parameter bit    FlushOnChar = 1
) (
  input               clk_i,
  input               rst_ni,

  input               req_i,
  input               we_i,
  input        [ 3:0] be_i,
  input        [31:0] addr_i,
  input        [31:0] wdata_i,
  output logic        rvalid_o,
  output logic [31:0] rdata_o
);

  localparam logic [7:0] CHAR_OUT_ADDR = 8'h0;
  localparam logic [7:0] SIM_CTRL_ADDR = 8'h2;
  localparam logic [7:0] CHAR_IN_ADDR  = 8'h4;

  logic [7:0] ctrl_addr;
  logic [2:0] sim_finish;
  logic [31:0] rdata_q;

  integer log_fd;
  integer stdin_fd;
  integer char_in;

  initial begin
    log_fd = $fopen(LogName, "w");
    if (log_fd == 0) begin
      $display("[sim_ctrl] FATAL: could not open log file %s", LogName);
    end else begin
      $display("[sim_ctrl] opened log %s", LogName);
    end

    stdin_fd = $fopen(InputFile, "r");
    if (stdin_fd == 0) begin
      $display("[sim_ctrl] FATAL: could not open input file %s", InputFile);
    end else begin
      $display("[sim_ctrl] opened input %s", InputFile);
    end
  end

  final begin
    $fclose(log_fd);
    $fclose(stdin_fd);
  end

  assign ctrl_addr = addr_i[9:2];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      rvalid_o <= 0;
      rdata_q  <= '0;
      sim_finish <= 'b0;
    end else begin
      rvalid_o <= req_i;
      rdata_q  <= '0;

      if (req_i & we_i) begin
        case (ctrl_addr)
          CHAR_OUT_ADDR: begin
            if (be_i[0]) begin
              $fwrite(log_fd, "%c", wdata_i[7:0]);
              if (FlushOnChar) $fflush(log_fd);
            end
          end
          SIM_CTRL_ADDR: begin
            if ((be_i[0] & wdata_i[0]) && (sim_finish == 'b0)) begin
              $display("Terminating simulation by software request.");
              sim_finish <= 3'b001;
            end
          end
          default: ;
        endcase
      end else if (req_i & ~we_i) begin
        case (ctrl_addr)
          CHAR_IN_ADDR: begin
            /* verilator lint_off BLKSEQ */
            char_in = $fgetc(stdin_fd);
            /* verilator lint_on BLKSEQ */
            rdata_q <= (char_in == -1) ? 32'hFF : {24'b0, char_in[7:0]};
          end
          default: ;
        endcase
      end
    end

    if (sim_finish != 'b0) begin
      sim_finish <= sim_finish + 1;
    end
    if (sim_finish >= 3'b010) begin
      $finish;
    end
  end

  assign rdata_o = rdata_q;
endmodule
