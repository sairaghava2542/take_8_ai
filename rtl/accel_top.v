// ============================================================================
// Accelerator Top Module
//
// Wraps the 8x8 systolic array with the bus register interface.
// Single clock domain - matches the RISC-V host protocol directly.
//
// Low-power partitioning:
//   PD_AON      : this top shell and u_regs, powered by VDD_ALW at 1.05 V.
//   PD_COMPUTE  : u_compute, powered by switched VDD_COMP at 0.78 V or 0 V.
//
// pwr_compute_en is generated in the always-on controller and is used by UPF
// as the compute-domain power-switch enable. The explicit boundary wires below
// give synthesis and implementation tools clear places to insert isolation and
// level-shifter cells.
// ============================================================================

module accel_top (
    input  wire        clk,
    input  wire        rst_n,

    // CPU data interface (rv32i_cpu compatible)
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        req,
    output wire [31:0] rdata,
    output wire        ready,

    // Interrupt and low-power control outputs
    output wire        irq,
    output wire        pwr_compute_en
);

    // AON-to-compute boundary signals.
    wire        arr_start;
    wire [63:0] arr_wgt_data;
    wire        arr_wgt_valid;
    wire [63:0] arr_act_data;
    wire        arr_act_valid;

    // Raw compute-to-AON outputs. UPF inserts isolation on these nets.
    wire         compute_busy_raw;
    wire         compute_done_raw;
    wire [255:0] compute_result_data_raw;
    wire         compute_result_valid_raw;
    wire [2:0]   compute_result_col_raw;

    // AON-visible, structurally clamped versions of compute outputs. These
    // mirror the UPF isolation policy and keep RTL simulation deterministic
    // when the compute island is logically off.
    wire         arr_busy;
    wire         arr_done;
    wire [255:0] arr_result_data;
    wire         arr_result_valid;
    wire [2:0]   arr_result_col;

    assign arr_busy         = pwr_compute_en ? compute_busy_raw         : 1'b0;
    assign arr_done         = pwr_compute_en ? compute_done_raw         : 1'b0;
    assign arr_result_data  = pwr_compute_en ? compute_result_data_raw  : 256'd0;
    assign arr_result_valid = pwr_compute_en ? compute_result_valid_raw : 1'b0;
    assign arr_result_col   = pwr_compute_en ? compute_result_col_raw   : 3'd0;

    // Reset for array. Soft-reset is reserved for a future CTRL bit expansion.
    wire arr_soft_reset;
    wire arr_rst_n = rst_n & ~arr_soft_reset;

    // Always-on bus register interface and streaming controller.
    accel_regs u_regs (
        .clk              (clk),
        .rst_n            (rst_n),
        .addr             (addr),
        .wdata            (wdata),
        .wstrb            (wstrb),
        .req              (req),
        .rdata            (rdata),
        .ready            (ready),
        .irq              (irq),
        .pwr_compute_en   (pwr_compute_en),
        .arr_start        (arr_start),
        .arr_busy         (arr_busy),
        .arr_done         (arr_done),
        .arr_wgt_data     (arr_wgt_data),
        .arr_wgt_valid    (arr_wgt_valid),
        .arr_act_data     (arr_act_data),
        .arr_act_valid    (arr_act_valid),
        .arr_result_data  (arr_result_data),
        .arr_result_valid (arr_result_valid),
        .arr_result_col   (arr_result_col)
    );

    assign arr_soft_reset = 1'b0;

    // Switchable compute voltage island.
    systolic_array_8x8 u_compute (
        .clk          (clk),
        .rst_n        (arr_rst_n),
        .start        (arr_start),
        .busy         (compute_busy_raw),
        .done         (compute_done_raw),
        .wgt_data     (arr_wgt_data),
        .wgt_valid    (arr_wgt_valid),
        .act_data     (arr_act_data),
        .act_valid    (arr_act_valid),
        .result_data  (compute_result_data_raw),
        .result_valid (compute_result_valid_raw),
        .result_col   (compute_result_col_raw)
    );

endmodule
