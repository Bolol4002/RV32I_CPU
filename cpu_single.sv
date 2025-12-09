module cpu_single (
    input  logic        clk,
    input  logic        reset
    );
    // internal wires
    logic [31:0] pc, next_pc, instr;
    logic [31:0] imm;
    logic [31:0] rs1_data, rs2_data;
    logic [4:0]  rs1, rs2, rd;
    logic [6:0]  opcode;
    logic [2:0]  funct3;
    logic [6:0]  funct7;

    // control signals
    logic        reg_write;
    logic        alu_src;
    logic        mem_read;
    logic        mem_write;
    logic        mem_to_reg;
    logic        branch;
    logic        jump;
    logic [1:0]  alu_op;

    // ALU control + ALU wires
    logic [3:0]  alu_ctrl;
    logic [31:0] alu_in_b, alu_result;
    logic [31:0] mem_rdata, wb_data;

    // instantiate PC
    pc_reg pc_i(.clk(clk), .reset(reset), .next_pc(next_pc), .pc_write(1'b1), .pc_out(pc));

    // instruction memory (assume imem is accessible for TB to fill)
    instruction_memory imem_i(.addr(pc), .instr(instr));

    // decode fields
    assign opcode = instr[6:0];
    assign rd     = instr[11:7];
    assign funct3 = instr[14:12];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign funct7 = instr[31:25];

    // imm generator and control unit (assumed to be your modules)
    imm_gen immgen(.instr(instr), .imm(imm));
    control_unit ctrl(.instr(instr),
                      .reg_write(reg_write),
                      .alu_src(alu_src),
                      .mem_read(mem_read),
                      .mem_write(mem_write),
                      .mem_to_reg(mem_to_reg),
                      .branch(branch),
                      .jump(jump),
                      .alu_op(alu_op));

    // regfile
    regfile rf(.clk(clk), .wen(reg_write),
               .waddr(rd), .wdata(wb_data),
               .raddr1(rs1), .raddr2(rs2),
               .rdata1(rs1_data), .rdata2(rs2_data));

    // ALU control
    alu_control actrl(.alu_op(alu_op), .funct3(funct3), .funct7(funct7), .alu_ctrl(alu_ctrl));

    // ALU input mux (immediate vs rs2)
    assign alu_in_b = (alu_src) ? imm : rs2_data;

    // ALU instantiation (assumes module name 'alu' with ports a,b,alu_op,result)
    alu myalu(.a(rs1_data), .b(alu_in_b), .alu_op(alu_ctrl), .result(alu_result));

    // Data memory
    data_memory dmem(.clk(clk), .mem_read(mem_read), .mem_write(mem_write), .addr(alu_result), .wdata(rs2_data), .rdata(mem_rdata));

    // writeback mux
    assign wb_data = (mem_to_reg) ? mem_rdata : alu_result;

    // Branch evaluation (supports common branch types using funct3)
    logic branch_taken;
    always_comb begin
        branch_taken = 1'b0;
        if (branch) begin
            unique case (funct3)
                3'b000: branch_taken = (rs1_data == rs2_data);               // beq
                3'b001: branch_taken = (rs1_data != rs2_data);               // bne
                3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data)); // blt
                3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // bge
                3'b110: branch_taken = (rs1_data < rs2_data);                // bltu
                3'b111: branch_taken = (rs1_data >= rs2_data);               // bgeu
                default: branch_taken = 1'b0;
            endcase
        end
    end

    // next_pc logic: priority: jal/jalr > branch > pc+4
    always_comb begin
        if (jump && (opcode == 7'b1101111)) begin // JAL
            next_pc = pc + imm;
        end else if (jump && (opcode == 7'b1100111)) begin // JALR
            next_pc = (rs1_data + imm) & ~32'd1;
        end else if (branch && branch_taken) begin
            next_pc = pc + imm;
        end else begin
            next_pc = pc + 4;
        end
    end
endmodule