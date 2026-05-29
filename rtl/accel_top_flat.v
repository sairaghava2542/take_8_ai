// ============================================================================
// Accelerator Top — Flat Single-Module (for custom synthesis flow)
//
// This is a FLATTENED version of the accelerator hierarchy:
//   accel_top → accel_regs + systolic_array_8x8 → 64× systolic_pe
//
// All sub-module logic is inlined into this single module so the synthesis engine
// can synthesize the entire design in one read_verilog → synth → export_sky130
// pass. Functionally identical to the hierarchical version.
//
// Architecture: 8×8 INT8 systolic array with output-stationary accumulation,
// memory-mapped register interface, diagonal-skewed streaming controller.
//
// CPU interface: rv32i_cpu native handshake (req/ready, addr, wdata, wstrb, rdata)
// Register Map:
//   0x000 CTRL   [W]  bit 0: start
//   0x004 STATUS [R]  bit 0: busy, 1: result_valid, 2: done(W1C), 3: irq_en
//   0x008 CONFIG [RW] bits[4:0]: compute_cycles, bit[8]: irq_en
//   0x100 A_MAT  [W]  16 words (8×8 activations, row-major, 4 bytes/word)
//   0x200 B_MAT  [W]  16 words (8×8 weights)
//   0x300 RESULT [R]  64 words (8×8 INT32 results)
// ============================================================================

module accel_top (
    input  wire        clk,
    input  wire        rst_n,

    // CPU data interface (rv32i_cpu compatible)
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        req,
    output reg  [31:0] rdata,
    output reg         ready,

    // Interrupt output
    output wire        irq
);

    // ====================================================================
    // SECTION 1: BUS REGISTER INTERFACE (from accel_regs)
    // ====================================================================

    // ── Input Matrix Buffers (flat 1D: index = row*8 + col) ─────────
    reg [7:0] a_buf [0:63];   // A matrix (activations)
    reg [7:0] b_buf [0:63];   // B matrix (weights)

    // ── Output Result Buffer (flat 1D: index = row*8 + col) ─────────
    reg [31:0] c_buf [0:63];  // C = A × B results

    // ── Control/Status ───────────────────────────────────────────────
    reg        done_sticky;
    reg        irq_en;
    reg        arr_done_prev;
    reg [4:0]  config_cycles;

    assign irq = done_sticky & irq_en;

    // ── Inter-section Signals (was inter-module wires) ───────────────
    reg         arr_start;
    wire        arr_busy;
    wire        arr_done;
    reg  [63:0] arr_wgt_data;
    reg         arr_wgt_valid;
    reg  [63:0] arr_act_data;
    reg         arr_act_valid;
    reg  [255:0] arr_result_data;    wire         arr_result_valid;
    wire [2:0]   arr_result_col;

    // ── Streaming Controller FSM ─────────────────────────────────────
    localparam SC_IDLE    = 3'd0,
               SC_LOAD    = 3'd1,
               SC_COMPUTE = 3'd2,
               SC_WAIT    = 3'd3;

    reg [2:0]  sc_state;
    reg [4:0]  sc_cnt;

    // ── Done Capture ─────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_sticky   <= 1'b0;
            arr_done_prev <= 1'b0;
        end else begin
            arr_done_prev <= arr_done;
            if (arr_done & ~arr_done_prev)
                done_sticky <= 1'b1;
            if (req & |wstrb & (addr[11:0] == 12'h004) & wdata[2])
                done_sticky <= 1'b0;
            if (arr_start)
                done_sticky <= 1'b0;
        end
    end

    // ── Result Capture (with async reset for c_buf) ─────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin : c_buf_reset
            integer ri;
            for (ri = 0; ri < 64; ri = ri + 1)
                c_buf[ri] <= 32'd0;
        end else if (arr_result_valid) begin
            c_buf[{3'd0, arr_result_col}] <= arr_result_data[0*32 +: 32];
            c_buf[{3'd1, arr_result_col}] <= arr_result_data[1*32 +: 32];
            c_buf[{3'd2, arr_result_col}] <= arr_result_data[2*32 +: 32];
            c_buf[{3'd3, arr_result_col}] <= arr_result_data[3*32 +: 32];
            c_buf[{3'd4, arr_result_col}] <= arr_result_data[4*32 +: 32];
            c_buf[{3'd5, arr_result_col}] <= arr_result_data[5*32 +: 32];
            c_buf[{3'd6, arr_result_col}] <= arr_result_data[6*32 +: 32];
            c_buf[{3'd7, arr_result_col}] <= arr_result_data[7*32 +: 32];
        end
    end

    // ── Streaming Controller ─────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sc_state      <= SC_IDLE;
            sc_cnt        <= 5'd0;
            arr_wgt_data  <= 64'd0;
            arr_act_data  <= 64'd0;
            arr_wgt_valid <= 1'b0;
            arr_act_valid <= 1'b0;
            arr_start     <= 1'b0;
        end else begin
            arr_start     <= 1'b0;
            arr_wgt_valid <= 1'b0;
            arr_act_valid <= 1'b0;

            case (sc_state)
                SC_IDLE: begin
                    if (req & |wstrb & (addr[11:0] == 12'h000) & wdata[0]) begin
                        arr_start <= 1'b1;
                        sc_state  <= SC_LOAD;
                        sc_cnt    <= 5'd0;
                    end
                end

                SC_LOAD: begin
                    arr_wgt_valid <= 1'b1;
                    begin : load_wgt_gen
                        integer ci;
                        for (ci = 0; ci < 8; ci = ci + 1)
                            arr_wgt_data[ci*8 +: 8] <= b_buf[{sc_cnt[2:0], ci[2:0]}];
                    end
                    sc_cnt <= sc_cnt + 5'd1;
                    if (sc_cnt == 5'd7) begin
                        sc_state <= SC_COMPUTE;
                        sc_cnt   <= 5'd0;
                    end
                end

                SC_COMPUTE: begin
                    arr_act_valid <= 1'b1;
                    arr_wgt_valid <= 1'b1;
                    begin : compute_skew_gen
                        integer ri, rj;
                        for (ri = 0; ri < 8; ri = ri + 1) begin
                            if (sc_cnt >= ri[4:0] && (sc_cnt - ri[4:0]) < 5'd8)
                                arr_act_data[ri*8 +: 8] <= a_buf[{ri[2:0], sc_cnt[2:0] - ri[2:0]}];
                            else
                                arr_act_data[ri*8 +: 8] <= 8'd0;
                        end
                        for (rj = 0; rj < 8; rj = rj + 1) begin
                            if (sc_cnt >= rj[4:0] && (sc_cnt - rj[4:0]) < 5'd8)
                                arr_wgt_data[rj*8 +: 8] <= b_buf[{sc_cnt[2:0] - rj[2:0], rj[2:0]}];
                            else
                                arr_wgt_data[rj*8 +: 8] <= 8'd0;
                        end
                    end
                    sc_cnt <= sc_cnt + 5'd1;
                    if (config_cycles != 5'd0 && sc_cnt == config_cycles - 5'd1) begin
                        sc_state <= SC_WAIT;
                        sc_cnt   <= 5'd0;
                    end
                end

                SC_WAIT: begin
                    if (arr_done)
                        sc_state <= SC_IDLE;
                end

                default: sc_state <= SC_IDLE;
            endcase
        end
    end

    // ── Bus Read/Write Logic ─────────────────────────────────────────
    wire [11:0] byte_addr = addr[11:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ready <= 1'b0;
        else
            ready <= req & ~ready;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_en        <= 1'b0;
            config_cycles <= 5'd22;
        end else if (req & |wstrb & ~ready) begin
            if (byte_addr == 12'h008) begin
                config_cycles <= wdata[4:0];
                irq_en        <= wdata[8];
            end
            if (byte_addr >= 12'h100 && byte_addr <= 12'h13C) begin
                begin : a_write
                    reg [3:0] widx;
                    reg [2:0] row;
                    reg       half;
                    widx = byte_addr[5:2];
                    row  = widx[3:1];
                    half = widx[0];
                    if (!half) begin
                        if (wstrb[0]) a_buf[{row, 3'd0}] <= wdata[7:0];
                        if (wstrb[1]) a_buf[{row, 3'd1}] <= wdata[15:8];
                        if (wstrb[2]) a_buf[{row, 3'd2}] <= wdata[23:16];
                        if (wstrb[3]) a_buf[{row, 3'd3}] <= wdata[31:24];
                    end else begin
                        if (wstrb[0]) a_buf[{row, 3'd4}] <= wdata[7:0];
                        if (wstrb[1]) a_buf[{row, 3'd5}] <= wdata[15:8];
                        if (wstrb[2]) a_buf[{row, 3'd6}] <= wdata[23:16];
                        if (wstrb[3]) a_buf[{row, 3'd7}] <= wdata[31:24];
                    end
                end
            end
            if (byte_addr >= 12'h200 && byte_addr <= 12'h23C) begin
                begin : b_write
                    reg [3:0] widx;
                    reg [2:0] row;
                    reg       half;
                    widx = byte_addr[5:2];
                    row  = widx[3:1];
                    half = widx[0];
                    if (!half) begin
                        if (wstrb[0]) b_buf[{row, 3'd0}] <= wdata[7:0];
                        if (wstrb[1]) b_buf[{row, 3'd1}] <= wdata[15:8];
                        if (wstrb[2]) b_buf[{row, 3'd2}] <= wdata[23:16];
                        if (wstrb[3]) b_buf[{row, 3'd3}] <= wdata[31:24];
                    end else begin
                        if (wstrb[0]) b_buf[{row, 3'd4}] <= wdata[7:0];
                        if (wstrb[1]) b_buf[{row, 3'd5}] <= wdata[15:8];
                        if (wstrb[2]) b_buf[{row, 3'd6}] <= wdata[23:16];
                        if (wstrb[3]) b_buf[{row, 3'd7}] <= wdata[31:24];
                    end
                end
            end
        end
    end

    always @(*) begin
        rdata = 32'd0;
        if (byte_addr == 12'h004)
            rdata = {28'd0, irq_en, done_sticky, arr_result_valid, arr_busy};
        else if (byte_addr == 12'h008)
            rdata = {23'd0, irq_en, 3'd0, config_cycles};
        else if (byte_addr >= 12'h300 && byte_addr <= 12'h3FC) begin
            begin : result_read
                reg [5:0] ridx;
                ridx = byte_addr[7:2];
                rdata = c_buf[ridx];
            end
        end
    end

    // ====================================================================
    // SECTION 2: ARRAY FSM (from systolic_array_8x8)
    // ====================================================================

    localparam [2:0] ARR_IDLE    = 3'd0,
                     ARR_LOAD    = 3'd1,
                     ARR_COMPUTE = 3'd2,
                     ARR_DRAIN   = 3'd3,
                     ARR_DONE    = 3'd4;

    reg [2:0] arr_state, arr_next_state;
    reg [4:0] arr_cycle_cnt;

    reg pe_weight_load;
    reg pe_compute_en;
    reg pe_acc_clear;
    reg pe_drain;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arr_state     <= ARR_IDLE;
            arr_cycle_cnt <= 5'd0;
        end else begin
            arr_state <= arr_next_state;
            if (arr_state != arr_next_state)
                arr_cycle_cnt <= 5'd0;
            else
                arr_cycle_cnt <= arr_cycle_cnt + 5'd1;
        end
    end

    always @(*) begin
        arr_next_state = arr_state;
        case (arr_state)
            ARR_IDLE:    if (arr_start) arr_next_state = ARR_LOAD;
            ARR_LOAD:    if (arr_cycle_cnt == 5'd7)  arr_next_state = ARR_COMPUTE;
            ARR_COMPUTE: if (arr_cycle_cnt == 5'd21) arr_next_state = ARR_DRAIN;
            ARR_DRAIN:   if (arr_cycle_cnt == 5'd8)  arr_next_state = ARR_DONE;
            ARR_DONE:    arr_next_state = ARR_IDLE;
            default:     arr_next_state = ARR_IDLE;
        endcase
    end

    always @(*) begin
        pe_weight_load = 1'b0;
        pe_compute_en  = 1'b0;
        pe_acc_clear   = 1'b0;
        pe_drain       = 1'b0;
        case (arr_state)
            ARR_IDLE:    if (arr_start) pe_acc_clear = 1'b1;
            ARR_LOAD:    pe_weight_load = 1'b1;
            ARR_COMPUTE: pe_compute_en = 1'b1;
            ARR_DRAIN:   pe_drain = 1'b1;
            default: ;
        endcase
    end

    assign arr_busy = (arr_state != ARR_IDLE) && (arr_state != ARR_DONE);
    assign arr_done = (arr_state == ARR_DONE);

    reg [2:0] arr_drain_col;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            arr_drain_col <= 3'd0;
        else if (arr_state == ARR_DRAIN)
            arr_drain_col <= arr_cycle_cnt[2:0];
        else
            arr_drain_col <= 3'd0;
    end

    assign arr_result_col   = arr_drain_col;
    assign arr_result_valid = (arr_state == ARR_DRAIN);

    // ====================================================================
    // SECTION 3: 64 PROCESSING ELEMENTS (from systolic_pe, inlined)
    // ====================================================================

    // PE interconnect wires
    wire [7:0] act_wire [0:7][0:8];  // activation: left→right (extra col for boundary)
    wire [7:0] wgt_wire [0:8][0:7];  // weight: top→bottom (extra row for boundary)

    // PE accumulator outputs
    wire [31:0] pe_acc_out [0:7][0:7];

    // Boundary connections — left side: activation inputs
    genvar gb_r, gb_c;
    generate
        for (gb_r = 0; gb_r < 8; gb_r = gb_r + 1) begin : gen_act_bnd
            assign act_wire[gb_r][0] = (arr_state == ARR_COMPUTE && arr_act_valid)
                                       ? arr_act_data[gb_r*8 +: 8] : 8'd0;
        end
    endgenerate

    // Boundary connections — top side: weight inputs
    generate
        for (gb_c = 0; gb_c < 8; gb_c = gb_c + 1) begin : gen_wgt_bnd
            assign wgt_wire[0][gb_c] = ((arr_state == ARR_LOAD && arr_wgt_valid) ||
                                        (arr_state == ARR_COMPUTE && arr_wgt_valid))
                                       ? arr_wgt_data[gb_c*8 +: 8] : 8'd0;
        end
    endgenerate

    // Drain mux — single always block, constant-only array indices (synthesis safe)
    always @(*) begin
        case (arr_drain_col)
            3'd0: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][0];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][0];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][0];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][0];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][0];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][0];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][0];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][0];
            end
            3'd1: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][1];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][1];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][1];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][1];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][1];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][1];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][1];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][1];
            end
            3'd2: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][2];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][2];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][2];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][2];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][2];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][2];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][2];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][2];
            end
            3'd3: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][3];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][3];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][3];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][3];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][3];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][3];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][3];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][3];
            end
            3'd4: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][4];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][4];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][4];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][4];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][4];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][4];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][4];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][4];
            end
            3'd5: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][5];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][5];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][5];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][5];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][5];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][5];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][5];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][5];
            end
            3'd6: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][6];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][6];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][6];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][6];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][6];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][6];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][6];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][6];
            end
            3'd7: begin
                arr_result_data[0*32 +: 32] = pe_acc_out[0][7];
                arr_result_data[1*32 +: 32] = pe_acc_out[1][7];
                arr_result_data[2*32 +: 32] = pe_acc_out[2][7];
                arr_result_data[3*32 +: 32] = pe_acc_out[3][7];
                arr_result_data[4*32 +: 32] = pe_acc_out[4][7];
                arr_result_data[5*32 +: 32] = pe_acc_out[5][7];
                arr_result_data[6*32 +: 32] = pe_acc_out[6][7];
                arr_result_data[7*32 +: 32] = pe_acc_out[7][7];
            end
            default: arr_result_data = 256'd0;
        endcase
    end

    // ── PE Grid (64 instances inlined via generate) ──────────────────
    genvar pe_r, pe_c;
    generate
        for (pe_r = 0; pe_r < 8; pe_r = pe_r + 1) begin : pe_row
            for (pe_c = 0; pe_c < 8; pe_c = pe_c + 1) begin : pe_col

                // Per-PE registers
                reg [7:0]        pe_weight_reg;
                reg [7:0]        pe_a_out;
                reg [7:0]        pe_w_out;
                reg signed [31:0] pe_accumulator;

                // Weight register
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n)
                        pe_weight_reg <= 8'd0;
                    else if (pe_weight_load)
                        pe_weight_reg <= wgt_wire[pe_r][pe_c];
                end

                // Activation pass-through (left → right)
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n)
                        pe_a_out <= 8'd0;
                    else if (pe_compute_en)
                        pe_a_out <= act_wire[pe_r][pe_c];
                end

                // Weight pass-through (top → bottom)
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n)
                        pe_w_out <= 8'd0;
                    else if (pe_weight_load || pe_compute_en)
                        pe_w_out <= wgt_wire[pe_r][pe_c];
                end

                // 8×8 signed multiply
                wire signed [7:0]  pe_a_signed  = $signed(act_wire[pe_r][pe_c]);
                wire signed [7:0]  pe_w_signed  = $signed(wgt_wire[pe_r][pe_c]);
                wire signed [15:0] pe_product   = pe_a_signed * pe_w_signed;
                wire signed [31:0] pe_prod_ext  = {{16{pe_product[15]}}, pe_product};
                wire signed [32:0] pe_sum_ext   = {pe_accumulator[31], pe_accumulator}
                                                + {pe_prod_ext[31], pe_prod_ext};

                // Overflow detection + saturation
                wire pe_ovf_pos = !pe_sum_ext[32] &&  pe_sum_ext[31];
                wire pe_ovf_neg =  pe_sum_ext[32] && !pe_sum_ext[31];
                wire signed [31:0] pe_sum_sat = pe_ovf_pos ? 32'h7FFF_FFFF :
                                                pe_ovf_neg ? 32'h8000_0000 :
                                                pe_sum_ext[31:0];

                // Accumulator
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n)
                        pe_accumulator <= 32'd0;
                    else if (pe_acc_clear)
                        pe_accumulator <= 32'd0;
                    else if (pe_compute_en)
                        pe_accumulator <= pe_sum_sat;
                end

                // Connect PE outputs to interconnect wires
                assign act_wire[pe_r][pe_c+1] = pe_a_out;
                assign wgt_wire[pe_r+1][pe_c] = pe_w_out;
                assign pe_acc_out[pe_r][pe_c] = pe_accumulator;

            end
        end
    endgenerate

endmodule
