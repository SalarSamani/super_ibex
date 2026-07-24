// Copyright 2026.
// SPDX-License-Identifier: Apache-2.0
//
// File-backed disk peripheral for Ibex Simple System.
//
// Memory map (1 KB window starting at the configured base address):
//   +0x00  CMD        (W)   1 = read sectors, 2 = write sectors
//   +0x04  SECTOR_LO  (RW)  low 32 bits of starting LBA
//   +0x08  SECTOR_HI  (RW)  high 32 bits of starting LBA (usually 0)
//   +0x0C  COUNT      (RW)  number of 512-byte sectors
//   +0x10  BUF_ADDR   (RW)  address in simulated RAM (in SoC space)
//   +0x14  STATUS     (R)   0 = done OK, 1 = error, 2 = busy (transient)
//
// On a CMD write the module performs the entire transfer in one cycle
// using $fseek/$fread/$fwrite into a host file (disk.img by default)
// and a hierarchical reference into the SoC RAM. STATUS is updated on
// the following cycle.

module ibex_simple_system_disk #(
  parameter string ImgName = "disk.img",
  // Base of the simulated RAM in SoC address space, used to compute the
  // word index into the RAM array.
  parameter logic [31:0] RamBase = 32'h0010_0000,
  // Depth of the RAM in 32-bit words; the disk refuses transfers that
  // would step outside this range.
  parameter int unsigned RamDepthW = 1024*1024/4
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        req_i,
  input  logic        we_i,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic [3:0]  be_i,
  input  logic [31:0] addr_i,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [31:0] wdata_i,
  output logic        rvalid_o,
  output logic [31:0] rdata_o,
  output logic        err_o
);

  // Register offsets inside the 1 KB window.
  localparam logic [7:0] OFF_CMD       = 8'h00;
  localparam logic [7:0] OFF_SECTOR_LO = 8'h04;
  localparam logic [7:0] OFF_SECTOR_HI = 8'h08;
  localparam logic [7:0] OFF_COUNT     = 8'h0C;
  localparam logic [7:0] OFF_BUF_ADDR  = 8'h10;
  localparam logic [7:0] OFF_STATUS    = 8'h14;

  // Word indices (byte offset >> 2) used as case labels.
  localparam logic [5:0] IDX_CMD       = 6'(OFF_CMD       >> 2);
  localparam logic [5:0] IDX_SECTOR_LO = 6'(OFF_SECTOR_LO >> 2);
  localparam logic [5:0] IDX_SECTOR_HI = 6'(OFF_SECTOR_HI >> 2);
  localparam logic [5:0] IDX_COUNT     = 6'(OFF_COUNT     >> 2);
  localparam logic [5:0] IDX_BUF_ADDR  = 6'(OFF_BUF_ADDR  >> 2);
  localparam logic [5:0] IDX_STATUS    = 6'(OFF_STATUS    >> 2);

  // Command codes.
  localparam logic [31:0] CMD_READ  = 32'h1;
  localparam logic [31:0] CMD_WRITE = 32'h2;

  // Status codes.
  localparam logic [31:0] STATUS_OK    = 32'h0;
  localparam logic [31:0] STATUS_ERROR = 32'h1;

  // Sector size in bytes. Standard 512-byte sectors.
  localparam int unsigned SECTOR_BYTES = 512;
  localparam int unsigned SECTOR_WORDS = SECTOR_BYTES / 4;

  // Persistent registers exposed to software.
  logic [31:0] reg_sector_lo;
  logic [31:0] reg_sector_hi;
  logic [31:0] reg_count;
  logic [31:0] reg_buf_addr;
  logic [31:0] reg_status;

  // Bus response pipeline.
  logic [31:0] rdata_d, rdata_q;
  logic        rvalid_d, rvalid_q;

  // Disk image file handle (opened at time zero, kept open).
  int          img_fd;

  initial begin
    img_fd = $fopen(ImgName, "r+b");
    if (img_fd == 0) begin
      $display("[disk] WARNING: could not open %s in r+b; trying w+b", ImgName);
      img_fd = $fopen(ImgName, "w+b");
    end
    if (img_fd == 0) begin
      $display("[disk] FATAL: could not open disk image %s", ImgName);
    end else begin
      $display("[disk] opened %s", ImgName);
    end
  end

  // --------------------------------------------------------------------
  // Read mux for software register reads.
  // --------------------------------------------------------------------
  always_comb begin
    unique case (addr_i[7:2])
      IDX_CMD:       rdata_d = 32'h0;          // CMD reads as 0
      IDX_SECTOR_LO: rdata_d = reg_sector_lo;
      IDX_SECTOR_HI: rdata_d = reg_sector_hi;
      IDX_COUNT:     rdata_d = reg_count;
      IDX_BUF_ADDR:  rdata_d = reg_buf_addr;
      IDX_STATUS:    rdata_d = reg_status;
      default:       rdata_d = 32'h0;
    endcase
  end

  assign rvalid_d = req_i;          // 1-cycle response for both R and W

  // --------------------------------------------------------------------
  // Main register file + command engine.
  // --------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    automatic int          word_idx;
    automatic int          n_words;
    automatic int          k;
    automatic int          rc;
    automatic logic [31:0] data_word;
    automatic longint      byte_off;

    if (!rst_ni) begin
      reg_sector_lo <= 32'h0;
      reg_sector_hi <= 32'h0;
      reg_count     <= 32'h0;
      reg_buf_addr  <= 32'h0;
      reg_status    <= STATUS_OK;
      rvalid_q      <= 1'b0;
      rdata_q       <= 32'h0;
    end else begin
      // Bus response defaults
      rvalid_q <= rvalid_d;
      rdata_q  <= rdata_d;

      // Register writes
      if (req_i && we_i) begin
        unique case (addr_i[7:2])
          IDX_SECTOR_LO: reg_sector_lo <= wdata_i;
          IDX_SECTOR_HI: reg_sector_hi <= wdata_i;
          IDX_COUNT:     reg_count     <= wdata_i;
          IDX_BUF_ADDR:  reg_buf_addr  <= wdata_i;

          IDX_CMD: begin
            // ---- Validate ----
            n_words  = int'(reg_count) * SECTOR_WORDS;
            byte_off = (longint'(reg_sector_hi) << 32) |
                       longint'(reg_sector_lo);
            byte_off = byte_off * SECTOR_BYTES;

            if (img_fd == 0) begin
              reg_status <= STATUS_ERROR;
            end else if (reg_buf_addr < RamBase) begin
              reg_status <= STATUS_ERROR;
            end else if (((reg_buf_addr - RamBase) >> 2) + n_words
                          > RamDepthW) begin
              reg_status <= STATUS_ERROR;
            end else begin
              word_idx = int'((reg_buf_addr - RamBase) >> 2);
              rc       = $fseek(img_fd, int'(byte_off), 0);
              if (rc != 0) begin
                reg_status <= STATUS_ERROR;
              end else if (wdata_i == CMD_READ) begin
                // ---- READ: disk -> RAM ----
                // The file holds little-endian bytes (the host wrote them
                // in LE order, and the Ibex CPU reads RAM in LE order).
                // $fread of a 32-bit variable populates it MSB-first from
                // the file, so we must byte-swap before storing to RAM
                // so that *(uint32_t*)addr in software sees the same byte
                // sequence that exists in the file.
                for (k = 0; k < n_words; k++) begin
                  if ($fread(data_word, img_fd) != 4) begin
                    // Past EOF: pad with zero so a brand-new image still works.
                    data_word = 32'h0;
                  end
                  ibex_simple_system.u_ram.u_ram.mem[word_idx + k] =
                      { data_word[7:0],   data_word[15:8],
                        data_word[23:16], data_word[31:24] };
                end
                reg_status <= STATUS_OK;
              end else if (wdata_i == CMD_WRITE) begin
                // ---- WRITE: RAM -> disk ----
                // Write the bytes in LE order (lowest-addressed byte first)
                // so the file is a faithful copy of the in-memory image and
                // can be inspected / pre-built by host-side tools like
                // lfs-fuse without any byte-swap step.
                for (k = 0; k < n_words; k++) begin
                  data_word = ibex_simple_system.u_ram.u_ram.mem[word_idx + k];
                  $fwrite(img_fd, "%c%c%c%c",
                          data_word[7:0],   data_word[15:8],
                          data_word[23:16], data_word[31:24]);
                end
                $fflush(img_fd);
                reg_status <= STATUS_OK;
              end else begin
                reg_status <= STATUS_ERROR;
              end
            end
          end

          default: ;
        endcase
      end
    end
  end

  assign rvalid_o = rvalid_q;
  assign rdata_o  = rdata_q;
  assign err_o    = 1'b0;

endmodule
