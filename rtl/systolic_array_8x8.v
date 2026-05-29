// ============================================================================
// 8×8 Systolic Array — INT8 Matrix Multiply Accelerator
//
// Architecture: Weight-stationary, output-stationary accumulation
//   - 64 PEs arranged in 8 rows × 8 columns
//   - Weights flow top→bottom (loaded column-wise during LOAD phase)
//   - Activations flow left→right (streamed row-wise during COMPUTE phase)
//   - Each PE accumulates partial sum locally (32-bit)
//   - Results drained column-wise during DRAIN phase
//
// Operation: C[8×8] = A[8×8] × B[8×8] (INT8 inputs, INT32 output)
//   IDLE → LOAD (8 cycles) → COMPUTE (22 cycles) → DRAIN (8 cycles) → IDLE
//
// Interface:
//   - act_in[7:0][7:0]   : 8 activation inputs (one per row)
//   - wgt_in[7:0][7:0]   : 8 weight inputs (one per column)
//   - result_out[31:0]    : drain output (column-by-column)
//   - result_col[2:0]     : which column is currently draining
//   - result_valid         : result data valid flag
// ============================================================================

module systolic_array_8x8 (
    input  wire        clk,
    input  wire        rst_n,

    // Control
    input  wire        start,          // begin operation (IDLE → LOAD)
    output wire        busy,           // array is operating
    output wire        done,           // operation complete, results ready

    // Weight loading (8 weights per cycle, 8 cycles = 64 weights)
    input  wire [63:0] wgt_data,       // 8 × 8-bit weights (packed)
    input  wire        wgt_valid,      // weight data valid

    // Activation streaming (8 activations per cycle during COMPUTE)
    input  wire [63:0] act_data,       // 8 × 8-bit activations (packed)
    input  wire        act_valid,      // activation data valid

    // Result drain (one column of 8 results per cycle)
    output wire [255:0] result_data,   // 8 × 32-bit results (packed)
    output wire         result_valid,
    output wire [2:0]   result_col     // current drain column index
);

    // ── FSM States ──────────────────────────────────────────────────────
    localparam [2:0] S_IDLE    = 3'd0,
                     S_LOAD    = 3'd1,  // Load weights (8 cycles)
                     S_COMPUTE = 3'd2,  // Stream activations + weights (22 cycles for 8×8)
                     S_DRAIN   = 3'd3,  // Read out results (8 cycles)
                     S_DONE    = 3'd4;

    reg [2:0] state, next_state;
    reg [4:0] cycle_cnt;  // max 22

    // ── PE Control Signals ──────────────────────────────────────────────
    reg        pe_weight_load;
    reg        pe_compute_en;
    reg        pe_acc_clear;
    reg        pe_drain;

    // ── PE Interconnect Wires ───────────────────────────────────────────
    // Activation wires: left→right (a_out[row][col] → a_in[row][col+1])
    wire [7:0] act_wire [0:7][0:8]; // extra column for boundary

    // Weight wires: top→bottom (w_out[row][col] → w_in[row+1][col])
    wire [7:0] wgt_wire [0:8][0:7]; // extra row for boundary

    // Accumulator outputs (for drain)
    wire [31:0] pe_acc_out [0:7][0:7];
    wire        pe_acc_valid [0:7][0:7];

    // ── Boundary Connections ────────────────────────────────────────────
    // Left boundary: activation inputs from external port
    genvar r, c;
    generate
        for (r = 0; r < 8; r = r + 1) begin : gen_act_boundary
            assign act_wire[r][0] = (state == S_COMPUTE && act_valid) ? act_data[r*8 +: 8] : 8'd0;
        end
    endgenerate

    // Top boundary: weight inputs from external port (during LOAD and COMPUTE)
    generate
        for (c = 0; c < 8; c = c + 1) begin : gen_wgt_boundary
            assign wgt_wire[0][c] = ((state == S_LOAD && wgt_valid) ||
                                     (state == S_COMPUTE && wgt_valid))
                                    ? wgt_data[c*8 +: 8] : 8'd0;
        end
    endgenerate

    // ── PE Grid Instantiation ───────────────────────────────────────────
    generate
        for (r = 0; r < 8; r = r + 1) begin : gen_row
            for (c = 0; c < 8; c = c + 1) begin : gen_col
                systolic_pe pe_inst (
                    .clk        (clk),
                    .rst_n      (rst_n),

                    // Activation: from left neighbor (or boundary)
                    .a_in       (act_wire[r][c]),
                    .a_out      (act_wire[r][c+1]),

                    // Weight: from top neighbor (or boundary)
                    .w_in       (wgt_wire[r][c]),
                    .w_out      (wgt_wire[r+1][c]),

                    // Accumulator
                    .acc_out    (pe_acc_out[r][c]),
                    .acc_valid  (pe_acc_valid[r][c]),

                    // Control (broadcast to all PEs)
                    .weight_load(pe_weight_load),
                    .compute_en (pe_compute_en),
                    .acc_clear  (pe_acc_clear),
                    .drain      (pe_drain)
                );
            end
        end
    endgenerate

    // ── FSM: State Register ─────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            cycle_cnt <= 5'd0;
        end else begin
            state <= next_state;
            if (state != next_state)
                cycle_cnt <= 5'd0;
            else
                cycle_cnt <= cycle_cnt + 5'd1;
        end
    end

    // ── FSM: Next State Logic ───────────────────────────────────────────
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start)
                    next_state = S_LOAD;
            end
            S_LOAD: begin
                // 8 cycles to load all 8 rows of weights
                if (cycle_cnt == 5'd7)
                    next_state = S_COMPUTE;
            end
            S_COMPUTE: begin
                // 22 cycles: 15 boundary input cycles + 7 propagation flush
                // Last boundary input at cycle 14, propagates 7 hops to PE[7][7]
                if (cycle_cnt == 5'd21)
                    next_state = S_DRAIN;
            end
            S_DRAIN: begin
                // 9 cycles: 8 valid data + 1 exit cycle
                // result_valid is combinational (state==S_DRAIN)
                // Column 7 read needs state to remain S_DRAIN
                if (cycle_cnt == 5'd8)
                    next_state = S_DONE;
            end
            S_DONE: begin
                next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    // ── FSM: Output Logic ───────────────────────────────────────────────
    always @(*) begin
        pe_weight_load = 1'b0;
        pe_compute_en  = 1'b0;
        pe_acc_clear   = 1'b0;
        pe_drain       = 1'b0;

        case (state)
            S_IDLE: begin
                if (start)
                    pe_acc_clear = 1'b1;  // Clear accumulators before load
            end
            S_LOAD: begin
                pe_weight_load = 1'b1;
            end
            S_COMPUTE: begin
                pe_compute_en = 1'b1;
            end
            S_DRAIN: begin
                pe_drain = 1'b1;
            end
            default: ;
        endcase
    end

    // ── Status Outputs ──────────────────────────────────────────────────
    assign busy = (state != S_IDLE) && (state != S_DONE);
    assign done = (state == S_DONE);

    // ── Result Drain ────────────────────────────────────────────────────
    // During DRAIN, output one column per cycle
    reg [2:0] drain_col;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            drain_col <= 3'd0;
        else if (state == S_DRAIN)
            drain_col <= cycle_cnt[2:0];
        else
            drain_col <= 3'd0;
    end

    assign result_col = drain_col;
    assign result_valid = (state == S_DRAIN);

    // Mux: select the column being drained
    generate
        for (r = 0; r < 8; r = r + 1) begin : gen_drain_mux
            assign result_data[r*32 +: 32] = pe_acc_out[r][drain_col];
        end
    endgenerate

endmodule
