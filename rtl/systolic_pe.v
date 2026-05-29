// ============================================================================
// Processing Element (PE) — 8×8 INT8 Systolic Array
// Weight-stationary architecture with 32-bit accumulator
//
// Data flow:
//   - Weights flow top→bottom (w_in → w_out), latched on weight_load
//   - Activations flow left→right (a_in → a_out), registered each cycle
//   - Partial sums accumulate locally, read out on drain
//
// Ports:
//   a_in[7:0]       — activation input from left neighbor (signed INT8)
//   w_in[7:0]       — weight input from top neighbor (signed INT8)
//   a_out[7:0]      — activation output to right neighbor (1-cycle latency)
//   w_out[7:0]      — weight output to bottom neighbor (1-cycle latency)
//   acc_out[31:0]   — accumulated result (valid when acc_valid asserted)
//   acc_valid        — accumulator output valid flag
//
// Control:
//   weight_load      — load w_in into weight register (preload phase)
//   compute_en       — enable MAC operation
//   acc_clear        — clear accumulator to zero
//   drain            — output accumulated result
// ============================================================================

module systolic_pe (
    input  wire        clk,
    input  wire        rst_n,

    // Data inputs
    input  wire [7:0]  a_in,        // activation from left (signed INT8)
    input  wire [7:0]  w_in,        // weight from top (signed INT8)

    // Data outputs
    output reg  [7:0]  a_out,       // activation to right (1-cycle delay)
    output reg  [7:0]  w_out,       // weight to bottom (1-cycle delay)

    // Accumulator output
    output wire [31:0] acc_out,
    output reg         acc_valid,

    // Control
    input  wire        weight_load, // latch weight from w_in
    input  wire        compute_en,  // enable MAC
    input  wire        acc_clear,   // reset accumulator
    input  wire        drain        // output result
);

    // ── Weight Register ─────────────────────────────────────────────────
    reg [7:0] weight_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            weight_reg <= 8'd0;
        else if (weight_load)
            weight_reg <= w_in;
    end

    // ── Data Pass-Through Registers ─────────────────────────────────────
    // Activation flows left→right with 1-cycle latency
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            a_out <= 8'd0;
        else if (compute_en)
            a_out <= a_in;
    end

    // Weight flows top→bottom: during weight_load AND compute phases
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            w_out <= 8'd0;
        else if (weight_load || compute_en)
            w_out <= w_in;
    end

    // ── 8×8 Signed Multiplier ───────────────────────────────────────────
    // Output-stationary mode: multiply streaming a_in × w_in
    // Weight-reuse mode: multiply a_in × weight_reg (preloaded)
    // Default: output-stationary (use w_in directly during compute)
    wire signed [7:0]  a_signed = $signed(a_in);
    wire signed [7:0]  w_compute = $signed(w_in);  // streaming weight
    wire signed [15:0] product  = a_signed * w_compute;

    // ── 32-bit Accumulator ──────────────────────────────────────────────
    // Accumulates signed products: range ±2^31
    // Saturation on overflow to prevent wrap-around corruption
    reg signed [31:0] accumulator;

    wire signed [31:0] product_ext = {{16{product[15]}}, product};
    wire signed [32:0] sum_extended = {accumulator[31], accumulator} + {product_ext[31], product_ext};

    // Overflow detection
    wire overflow_pos = !sum_extended[32] &&  sum_extended[31]; // was positive, went negative
    wire overflow_neg =  sum_extended[32] && !sum_extended[31]; // was negative, went positive

    // Saturated sum
    wire signed [31:0] sum_saturated = overflow_pos ? 32'h7FFF_FFFF :  // +2^31 - 1
                                       overflow_neg ? 32'h8000_0000 :  // -2^31
                                       sum_extended[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            accumulator <= 32'd0;
        else if (acc_clear)
            accumulator <= 32'd0;
        else if (compute_en)
            accumulator <= sum_saturated;
    end

    // ── Output ──────────────────────────────────────────────────────────
    assign acc_out = accumulator;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acc_valid <= 1'b0;
        else
            acc_valid <= drain;
    end

endmodule
