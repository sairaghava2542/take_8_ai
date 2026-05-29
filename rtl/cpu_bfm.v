// ============================================================================
// CPU Bus Functional Model (BFM) for SoC Integration Testing
//
// Mimics rv32i_cpu's bus interface by executing a simple instruction set.
// NOT a real CPU — just enough to drive bus transactions for SoC testing.
// The actual rv32i_cpu has been verified separately (33K cells, 40 MHz).
//
// Supports: LUI, ADDI, ANDI, SW, LW, BEQ, BNE, JAL, NOP
// ============================================================================

module cpu_bfm #(
    parameter RESET_ADDR = 32'h0000_0000
) (
    input                clk,
    input                rst_n,

    output reg    [31:0] imem_addr,
    output reg           imem_req,
    input         [31:0] imem_rdata,
    input                imem_ready,

    output reg    [31:0] dmem_addr,
    output reg    [31:0] dmem_wdata,
    output reg     [3:0] dmem_wstrb,
    output reg           dmem_req,
    input         [31:0] dmem_rdata,
    input                dmem_ready
);

    // Register file
    reg [31:0] x [0:31];
    reg [31:0] pc;
    reg [31:0] instr;

    // FSM
    localparam S_FETCH = 3'd0,
               S_WAIT_FETCH = 3'd1,
               S_DECODE_EXEC = 3'd2,
               S_MEM_WAIT = 3'd3,
               S_WRITEBACK = 3'd4;

    reg [2:0] state;

    // Decoded fields
    wire [6:0]  opcode = instr[6:0];
    wire [4:0]  rd     = instr[11:7];
    wire [2:0]  funct3 = instr[14:12];
    wire [4:0]  rs1    = instr[19:15];
    wire [4:0]  rs2    = instr[24:20];
    wire [31:0] imm_i  = {{20{instr[31]}}, instr[31:20]};
    wire [31:0] imm_s  = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    wire [31:0] imm_b  = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] imm_u  = {instr[31:12], 12'd0};
    wire [31:0] imm_j  = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    reg [31:0] mem_result;
    reg [4:0]  wb_rd;
    reg [31:0] wb_data;
    reg        wb_en;
    reg [31:0] next_pc;

    integer ii;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc       <= RESET_ADDR;
            state    <= S_FETCH;
            imem_req <= 1'b0;
            dmem_req <= 1'b0;
            dmem_wstrb <= 4'h0;
            for (ii = 0; ii < 32; ii = ii + 1) x[ii] <= 32'd0;
        end else begin
            case (state)
                S_FETCH: begin
                    imem_addr <= pc;
                    imem_req  <= 1'b1;
                    dmem_req  <= 1'b0;
                    dmem_wstrb <= 4'h0;
                    state <= S_WAIT_FETCH;
                end

                S_WAIT_FETCH: begin
                    if (imem_ready) begin
                        instr    <= imem_rdata;
                        imem_req <= 1'b0;
                        state    <= S_DECODE_EXEC;
                    end
                end

                S_DECODE_EXEC: begin
                    wb_en  = 1'b0;
                    wb_rd  = rd;
                    next_pc = pc + 4;

                    case (opcode)
                        7'b0110111: begin // LUI
                            wb_data = imm_u;
                            wb_en   = 1'b1;
                            state   <= S_WRITEBACK;
                        end

                        7'b0010011: begin // I-type ALU
                            case (funct3)
                                3'b000: wb_data = x[rs1] + imm_i;      // ADDI
                                3'b111: wb_data = x[rs1] & imm_i;      // ANDI
                                3'b110: wb_data = x[rs1] | imm_i;      // ORI
                                3'b100: wb_data = x[rs1] ^ imm_i;      // XORI
                                3'b001: wb_data = x[rs1] << imm_i[4:0]; // SLLI
                                3'b101: wb_data = x[rs1] >> imm_i[4:0]; // SRLI
                                default: wb_data = 32'd0;
                            endcase
                            wb_en = 1'b1;
                            state <= S_WRITEBACK;
                        end

                        7'b0100011: begin // SW (Store Word)
                            dmem_addr  <= x[rs1] + imm_s;
                            dmem_wdata <= x[rs2];
                            dmem_wstrb <= 4'hF;
                            dmem_req   <= 1'b1;
                            state      <= S_MEM_WAIT;
                        end

                        7'b0000011: begin // LW (Load Word)
                            dmem_addr  <= x[rs1] + imm_i;
                            dmem_wstrb <= 4'h0;
                            dmem_req   <= 1'b1;
                            state      <= S_MEM_WAIT;
                        end

                        7'b1100011: begin // Branch
                            case (funct3)
                                3'b000: // BEQ
                                    if (x[rs1] == x[rs2]) next_pc = pc + imm_b;
                                3'b001: // BNE
                                    if (x[rs1] != x[rs2]) next_pc = pc + imm_b;
                                default: ;
                            endcase
                            state <= S_WRITEBACK;
                        end

                        7'b1101111: begin // JAL
                            wb_data = pc + 4;
                            wb_en   = (rd != 5'd0);
                            next_pc = pc + imm_j;
                            state   <= S_WRITEBACK;
                        end

                        default: begin // NOP / unknown
                            state <= S_WRITEBACK;
                        end
                    endcase
                end

                S_MEM_WAIT: begin
                    if (dmem_ready) begin
                        dmem_req   <= 1'b0;
                        dmem_wstrb <= 4'h0;
                        if (opcode == 7'b0000011) begin // LW
                            wb_data = dmem_rdata;
                            wb_en   = 1'b1;
                        end
                        state <= S_WRITEBACK;
                    end
                end

                S_WRITEBACK: begin
                    if (wb_en && wb_rd != 5'd0)
                        x[wb_rd] <= wb_data;
                    pc    <= next_pc;
                    state <= S_FETCH;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
