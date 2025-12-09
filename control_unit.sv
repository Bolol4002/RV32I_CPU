module control_unit(
    input  logic [31:0] instr,       // full instruction
    output logic        reg_write,
    output logic        alu_src,
    output logic        mem_read,
    output logic        mem_write,
    output logic        mem_to_reg,
    output logic        branch,
    output logic        jump,
    output logic [1:0]  alu_op
);

    logic [6:0] opcode;
    assign opcode = instr[6:0];

    always_comb begin
        // Default everything to zero
        reg_write  = 0;
        alu_src    = 0;
        mem_read   = 0;
        mem_write  = 0;
        mem_to_reg = 0;
        branch     = 0;
        jump       = 0;
        alu_op     = 2'b00;

        case (opcode)

            // -------------------------
            // R-TYPE (add, sub, and, or, xor, slt...)
            // opcode = 0110011
            // -------------------------
            7'b0110011: begin
                reg_write  = 1;
                alu_src    = 0;
                mem_to_reg = 0;
                alu_op     = 2'b10;     // R-type ALU ops
            end

            // -------------------------
            // I-TYPE ALU (addi, xori, andi, ori, slti...)
            // opcode = 0010011
            // -------------------------
            7'b0010011: begin
                reg_write  = 1;
                alu_src    = 1;
                mem_to_reg = 0;
                alu_op     = 2'b11;     // I-type ALU ops
            end

            // -------------------------
            // LOAD (lw)
            // opcode = 0000011
            // -------------------------
            7'b0000011: begin
                reg_write  = 1;
                alu_src    = 1;
                mem_read   = 1;
                mem_to_reg = 1;
                alu_op     = 2'b00;     // ADD for address calc
            end

            // -------------------------
            // STORE (sw)
            // opcode = 0100011
            // -------------------------
            7'b0100011: begin
                alu_src    = 1;
                mem_write  = 1;
                alu_op     = 2'b00;     // ADD for address calc
            end

            // -------------------------
            // BRANCH (beq, bne, blt...)
            // opcode = 1100011
            // -------------------------
            7'b1100011: begin
                branch     = 1;
                alu_src    = 0;
                alu_op     = 2'b01;     // SUB for compare
            end

            // -------------------------
            // JAL (jump and link)
            // opcode = 1101111
            // -------------------------
            7'b1101111: begin
                reg_write  = 1;
                jump       = 1;
                alu_op     = 2'b00;
            end

            // -------------------------
            // JALR
            // opcode = 1100111
            // -------------------------
            7'b1100111: begin
                reg_write  = 1;
                jump       = 1;
                alu_src    = 1;
                alu_op     = 2'b00;
            end

            // -------------------------
            // LUI (load upper immediate)
            // opcode = 0110111
            // -------------------------
            7'b0110111: begin
                reg_write  = 1;
                alu_src    = 1;         // immediate
                mem_to_reg = 0;
                alu_op     = 2'b00;     // ALU just passes imm
            end

            // -------------------------
            // AUIPC (add upper immediate to PC)
            // opcode = 0010111
            // -------------------------
            7'b0010111: begin
                reg_write  = 1;
                alu_src    = 1;
                alu_op     = 2'b00;     // ADD PC + imm
            end

            default: begin
                // CPU does nothing on unknown opcode
            end

        endcase
    end

endmodule
