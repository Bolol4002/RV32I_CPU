module alu_control(
    input  logic [1:0]  alu_op,     // from control_unit
    input  logic [2:0]  funct3,
    input  logic [6:0]  funct7,
    output logic [3:0]  alu_ctrl
    );
    // ALU encoding used by your alu module
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLT  = 4'b0101;
    localparam ALU_SLTU = 4'b0110;
    localparam ALU_SLL  = 4'b0111;
    localparam ALU_SRL  = 4'b1000;
    localparam ALU_SRA  = 4'b1001;

    always_comb begin
        alu_ctrl = ALU_ADD; // default

        case (alu_op)
            2'b00: begin // load/store / add (address calc)
                alu_ctrl = ALU_ADD;
            end
            2'b01: begin // branch -> compare (use SUB)
                alu_ctrl = ALU_SUB;
            end
            2'b10: begin // R-type: use funct3/funct7
                unique case (funct3)
                    3'b000: alu_ctrl = (funct7 == 7'b0100000) ? ALU_SUB : ALU_ADD;
                    3'b111: alu_ctrl = ALU_AND;
                    3'b110: alu_ctrl = ALU_OR;
                    3'b100: alu_ctrl = ALU_XOR;
                    3'b010: alu_ctrl = ALU_SLT;
                    3'b011: alu_ctrl = ALU_SLTU;
                    3'b001: alu_ctrl = ALU_SLL;
                    3'b101: alu_ctrl = (funct7 == 7'b0100000) ? ALU_SRA : ALU_SRL;
                    default: alu_ctrl = ALU_ADD;
                endcase
            end
            2'b11: begin // I-type arithmetic (funct3 decides; shifts use funct7 similarly)
                unique case (funct3)
                    3'b000: alu_ctrl = ALU_ADD; // addi
                    3'b111: alu_ctrl = ALU_AND;
                    3'b110: alu_ctrl = ALU_OR;
                    3'b100: alu_ctrl = ALU_XOR;
                    3'b010: alu_ctrl = ALU_SLT; // slti
                    3'b011: alu_ctrl = ALU_SLTU; // sltiu
                    3'b001: alu_ctrl = ALU_SLL; // slli (funct7==0000000)
                    3'b101: alu_ctrl = (funct7 == 7'b0100000) ? ALU_SRA : ALU_SRL; // srli/srai
                    default: alu_ctrl = ALU_ADD;
                endcase
            end
            default: alu_ctrl = ALU_ADD;
        endcase
    end
endmodule