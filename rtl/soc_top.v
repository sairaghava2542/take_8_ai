// ============================================================================
// SoC Top Level — RV32IM CPU + 8×8 INT8 Systolic Array Accelerator
//
// Memory Map:
//   0x0000_0000 - 0x0000_FFFF  IMEM (64 KB instruction memory, read-only)
//   0x0001_0000 - 0x0001_FFFF  DMEM (64 KB data memory, read/write)
//   0x4000_0000 - 0x4000_0FFF  ACCEL (accelerator registers, 4 KB)
//
// Address Decoding:
//   imem_addr → always routed to IMEM
//   dmem_addr[31:28] == 4'h0 → DMEM
//   dmem_addr[31:28] == 4'h4 → ACCEL
//
// Single clock domain. Both CPU and accelerator run at same frequency.
// ============================================================================

module soc_top #(
    parameter RESET_ADDR = 32'h0000_0000,
    parameter IMEM_WORDS = 16384,   // 64 KB
    parameter DMEM_WORDS = 16384    // 64 KB
) (
    input  wire        clk,
    input  wire        rst_n,

    // External interrupt (active high)
    output wire        accel_irq,

    // Debug / testbench interface
    output wire [31:0] dmem_addr_mon,   // monitor dmem address
    output wire        dmem_req_mon     // monitor dmem request
);

    // ── CPU Signals ──────────────────────────────────────────────────
    wire [31:0] imem_addr,  imem_rdata;
    wire        imem_req,   imem_ready;
    wire [31:0] dmem_addr,  dmem_wdata, dmem_rdata;
    wire [3:0]  dmem_wstrb;
    wire        dmem_req,   dmem_ready;

    // ── Address Decode ───────────────────────────────────────────────
    wire sel_dmem  = (dmem_addr[31:28] == 4'h0);
    wire sel_accel = (dmem_addr[31:28] == 4'h4);

    // ── DMEM Signals ─────────────────────────────────────────────────
    wire [31:0] dmem_mem_rdata;
    wire        dmem_mem_ready;

    // ── ACCEL Signals ────────────────────────────────────────────────
    wire [31:0] accel_rdata;
    wire        accel_ready;

    // ── CPU (BFM for simulation, rv32i_cpu for synthesis) ──────────────
    // The real rv32i_cpu has Icarus-incompatible forward references.
    // Use cpu_bfm for SoC integration testing; real CPU verified separately.
`ifdef USE_REAL_CPU
    rv32i_cpu #(
        .RESET_ADDR (RESET_ADDR)
    ) u_cpu (
`else
    cpu_bfm #(
        .RESET_ADDR (RESET_ADDR)
    ) u_cpu (
`endif
        .clk        (clk),
        .rst_n      (rst_n),
        .imem_addr  (imem_addr),
        .imem_req   (imem_req),
        .imem_rdata (imem_rdata),
        .imem_ready (imem_ready),
        .dmem_addr  (dmem_addr),
        .dmem_wdata (dmem_wdata),
        .dmem_wstrb (dmem_wstrb),
        .dmem_req   (dmem_req),
        .dmem_rdata (dmem_rdata),
        .dmem_ready (dmem_ready)
    );

    // ── Instruction Memory (ROM-like, single-port read) ──────────────
    reg [31:0] imem [0:IMEM_WORDS-1];
    reg [31:0] imem_rdata_r;
    reg        imem_ready_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imem_rdata_r <= 32'd0;
            imem_ready_r <= 1'b0;
        end else begin
            imem_ready_r <= imem_req;
            if (imem_req)
                imem_rdata_r <= imem[imem_addr[15:2]];
        end
    end
    assign imem_rdata = imem_rdata_r;
    assign imem_ready = imem_ready_r;

    // ── Data Memory (SRAM-like, read/write) ──────────────────────────
    reg [31:0] dmem_mem [0:DMEM_WORDS-1];
    reg [31:0] dmem_mem_rdata_r;
    reg        dmem_mem_ready_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmem_mem_rdata_r <= 32'd0;
            dmem_mem_ready_r <= 1'b0;
        end else begin
            dmem_mem_ready_r <= dmem_req & sel_dmem;
            if (dmem_req & sel_dmem) begin
                // Read
                dmem_mem_rdata_r <= dmem_mem[dmem_addr[15:2]];
                // Write (byte-enable)
                if (dmem_wstrb[0]) dmem_mem[dmem_addr[15:2]][7:0]   <= dmem_wdata[7:0];
                if (dmem_wstrb[1]) dmem_mem[dmem_addr[15:2]][15:8]  <= dmem_wdata[15:8];
                if (dmem_wstrb[2]) dmem_mem[dmem_addr[15:2]][23:16] <= dmem_wdata[23:16];
                if (dmem_wstrb[3]) dmem_mem[dmem_addr[15:2]][31:24] <= dmem_wdata[31:24];
            end
        end
    end
    assign dmem_mem_rdata = dmem_mem_rdata_r;
    assign dmem_mem_ready = dmem_mem_ready_r;

    // ── Accelerator ──────────────────────────────────────────────────
    accel_top u_accel (
        .clk    (clk),
        .rst_n  (rst_n),
        .addr   (dmem_addr),
        .wdata  (dmem_wdata),
        .wstrb  (dmem_wstrb & {4{sel_accel}}),
        .req    (dmem_req & sel_accel),
        .rdata  (accel_rdata),
        .ready  (accel_ready),
        .irq    (accel_irq)
    );

    // ── Data Bus Mux ─────────────────────────────────────────────────
    assign dmem_rdata = sel_accel ? accel_rdata : dmem_mem_rdata;
    assign dmem_ready = sel_accel ? accel_ready : dmem_mem_ready;

    // ── Debug Monitors ───────────────────────────────────────────────
    assign dmem_addr_mon = dmem_addr;
    assign dmem_req_mon  = dmem_req;

    // ── Memory Initialization (for simulation) ───────────────────────
    // Testbench loads imem/dmem via $readmemh or direct assignment
    // Format: imem[addr] = instruction

endmodule
