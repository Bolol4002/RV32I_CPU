# RV32I_CPU

---



## **MODULE 1 — regfile.sv**

### What it does
- Stores the 32 general-purpose registers (x0–x31).
- Provides two read ports (rs1, rs2) and one write port (rd).
- Writes occur on the rising edge of clk when wen is high.
- Reads are combinational: reading a register returns its current contents immediately.
- Enforces x0 = 0 (writes to x0 ignored; reads return 0).
### Why it matters
- Registers are the working set of the CPU — almost everything (ALU input operands, writebacks) flows through the regfile.
- Combinational reads let the rest of the datapath (decode and ALU) get operand values in the same cycle for a single-cycle design.
- Protecting x0 is part of the RISC-V architectural guarantee.
### How it plugs in
- ID stage supplies raddr1 and raddr2 to get operands for execution.
- WB stage supplies waddr, wdata, and wen to write results back.
- For pipelined design later, reads may come from ID pipeline register, and writes happen in WB stage (with forwarding to support hazards).
    


```
module regfile (
    input  wire        clk,
    input  wire        wen,
    input  wire [4:0]  waddr,
    input  wire [31:0] wdata,
    input  wire [4:0]  raddr1,
    input  wire [4:0]  raddr2,
    output wire [31:0] rdata1,
    output wire [31:0] rdata2
);

    // 32 registers, each 32 bits
    reg [31:0] mem [31:0];

    // synchronous write
    always @(posedge clk) begin
        if (wen && (waddr != 5'd0)) begin
            mem[waddr] <= wdata;  // x0 is hard-wired to zero
        end
    end

    // combinational read
    assign rdata1 = (raddr1 == 5'd0) ? 32'd0 : mem[raddr1];
    assign rdata2 = (raddr2 == 5'd0) ? 32'd0 : mem[raddr2];

endmodule
```

---

## **MODULE 2 — alu.sv**

### What it does
- Implements arithmetic and bitwise operations on 32-bit inputs a and b.
- Supports: ADD, SUB, AND, OR, XOR, SLT (signed), SLTU (unsigned), SLL, SRL, SRA.
- Shift amount is b[4:0] (only lower 5 bits used).
### Why it matters
- The ALU is the execution core — everything that computes values (arithmetic, logic, compare, address calc) happens here.
- For loads/stores and branches, ALU computes effective addresses and comparisons.

### How it plugs in
- Inputs come from register file (rs1, rs2) or rs1 + immediate depending on alu_src.
- Output either goes to data_memory for stores/addresses passed or to wb for register writeback.
- ALU selection (which operation to perform) is decided by an ALU control block (not the coarse control_unit).

    
    ```
    module alu (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic [3:0]  alu_op,   // 4 bits = enough for RV32I set
    output logic [31:0] result
    );

    always_comb begin
        case (alu_op)
            4'b0000: result = a + b;                      // ADD
            4'b0001: result = a - b;                      // SUB
            4'b0010: result = a & b;                      // AND
            4'b0011: result = a | b;                      // OR
            4'b0100: result = a ^ b;                      // XOR
            4'b0101: result = ($signed(a) < $signed(b));  // SLT
            4'b0110: result = (a < b);                    // SLTU
            4'b0111: result = a << b[4:0];                // SLL
            4'b1000: result = a >> b[4:0];                // SRL
            4'b1001: result = $signed(a) >>> b[4:0];      // SRA
            default: result = 32'd0;
        endcase
    end

    endmodule
    ```

---

## **MODULE 3 — imm_gen.sv**

### What it does
- Extracts and sign-extends immediates from the 32-bit instruction according to the instruction format:
- I-type: instr[31:20] → sign-extend to 32 bits (used by addi, lw, jalr etc.)
- S-type: instr[31:25] & instr[11:7] concatenated → sign-extend (stores)
- B-type: branch immediate built from instr[31], instr[7], instr[30:25], instr[11:8] with LSB = 0 → sign-extend (branch offset)
- U-type: top 20 bits instr[31:12] then 12 zeros (lui/auipc)
- J-type: JAL immediate built from scattered fields, LSB = 0 → sign-extend

### Why it matters
- The ALU often needs an immediate value for arithmetic and address calculations.
- Branch and jump PC targets are computed using immediates.
- Correct sign extension is critical: many bugs (wrong branch targets, wrong load addresses) come from incorrect imm bits or sign-extension.

### How it plugs in
- ID stage uses imm_gen to produce the immediate which is then fed either directly to the ALU (if alu_src=1) or used in PC calculation for branches/jumps.
- For lui and auipc, the immediate is treated specially (upper bits).

```
module imm_gen(
    input  logic [31:0] instr,
    output logic [31:0] imm
);

    logic [6:0] opcode;
    assign opcode = instr[6:0];

    always_comb begin
        case (opcode)

            // I-type: addi, lw, jalr, etc.
            7'b0010011,
            7'b0000011,
            7'b1100111: begin
                imm = {{20{instr[31]}}, instr[31:20]};
            end

            // S-type: sw, sh, sb
            7'b0100011: begin
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end

            // B-type: beq, bne, blt, bge
            7'b1100011: begin
                imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end

            // U-type: lui, auipc
            7'b0110111,
            7'b0010111: begin
                imm = {instr[31:12], 12'b0};
            end

            // J-type: jal
            7'b1101111: begin
                imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end

            default: imm = 32'd0;

        endcase
    end

endmodule

```
    

---

## **MODULE 4 — control_unit.sv**

### What it does
- Takes the 32-bit instruction (primarily opcode = instr[6:0]) and sets the top-level control signals that steer datapath behavior:
    - reg_write, alu_src, mem_read, mem_write, mem_to_reg, branch, jump, and a coarse alu_op.
- It maps instruction classes (R-type, I-type, load, store, branch, jal/jalr, lui, auipc) to the CPU actions they require.

### Why it matters
- Central decision-maker: without these signals you can’t know whether to writeback, whether to use immediate, whether to access memory, or whether to take a branch.
- Keeps instruction-class logic centralized and easy to audit.

### How it plugs in
- ID stage uses its outputs to:
    - Decide whether WB will write to a register.
    - Select ALU operand source (alu_src).
    - Activate memory interface for load/store.
    - Indicate branches/jumps to PC logic.
- alu_op is a coarse hint (example encodings in your file: 00=ADD, 01=SUB-for-compare, 10=R-type, 11=I-type). That is not the final ALU opcode — it’s fed to ALU Control.

### Important practical notes
- Two-level decoding: This is standard — control_unit (opcode-level) + alu_control (funct fields) → final ALU control. Implement alu_control next.
- jump and branch are different: branch requires a condition check by ALU (SUB/compare) before PC update; jump is unconditional (jal) — but jalr uses the register + imm, so it has alu_src=1 too.

```
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
```

---

## **MODULE 5 — instruction_memory.sv & data_memory.sv**

These can be simple behavioral memories.

### instruction_memory

**Role:** Read-only array loaded from hex file.

```
module instruction_memory #(
    parameter DEPTH = 256
    ) (
    input  logic [31:0] addr,   // byte address
    output logic [31:0] instr
    );
    // word addressed memory (DEPTH words)
    reg [31:0] imem [0:DEPTH-1];

    // read combinational — assume word-aligned PC
    assign instr = imem[addr[31:2]];

    // helper to load in TB: $readmemh can be used externally
endmodule
```

### data_memory

**Role:** Simple byte-addressable memory.  
Supports lw/sw fully.

**When this passes:**

- You can load/store values using a small test.

**data memory**
```
module data_memory #(
    parameter SIZE_BYTES = 1024
    ) (
    input  logic         clk,
    input  logic         mem_read,   // 1 = read
    input  logic         mem_write,  // 1 = write (word)
    input  logic [31:0]  addr,       // byte address
    input  logic [31:0]  wdata,      // write data (word)
    output logic [31:0]  rdata
    );
    // byte-addressable memory
    reg [7:0] dmem [0:SIZE_BYTES-1];

    // combinational read (word-aligned)
    always_comb begin
        if (mem_read) begin
            // little-endian assemble
            rdata = { dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr+0] };
        end else begin
            rdata = 32'd0;
        end
    end

    // synchronous write (store word). For simplicity, assume aligned addresses.
    always_ff @(posedge clk) begin
        if (mem_write) begin
            dmem[addr+0] <= wdata[7:0];
            dmem[addr+1] <= wdata[15:8];
            dmem[addr+2] <= wdata[23:16];
            dmem[addr+3] <= wdata[31:24];
        end
    end
endmodule
```
    

---

## **MODULE 6 — pc_reg.sv**

**Role:** Program counter register.

**Inputs:** clk, reset, next_pc  
**Output:** pc_out

**When this passes:**

- PC increments properly.
    
- PC jumps/branches correctly.
```
module pc_reg (
    input  logic        clk,
    input  logic        reset,     // synchronous reset (active high)
    input  logic [31:0] next_pc,
    input  logic        pc_write,  // when 0, stall (not used in single-cycle)
    output logic [31:0] pc_out
)   ;
    always_ff @(posedge clk) begin
        if (reset) pc_out <= 32'd0;
        else if (pc_write) pc_out <= next_pc;
    end
endmodule
```
    

## **MODULE 7 - alu_control.sv**
```
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
```


---


## **MODULE 8 — cpu_single.sv (top of single-cycle core)**
This instantiates everything:

```
PC → Instruction Memory → Decode → Register File
                            ↓          ↑
                       Immediate       |
                            ↓          |
                  Control Unit         |
                            ↓          |
                     ALU <-------------
                            ↓
                     Data Memory
                            ↓
                       Writeback
```

**Your job here:**
- Wire all modules together.
- Compute `next_pc` based on branch/jump logic.
- Write to register file when required.
    
**When this passes:**
- You can run a tiny program:
    - addi x1, x0, 5     
    - addi x2, x0, 7    
    - add x3, x1, x2    
    - store result → memory    
    - halt
Once this executes correctly → your single-cycle CPU is complete.

```
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
```

---

### Summary

| Module              | Role                            | Why It Matters                          |
| ------------------- | ------------------------------- | --------------------------------------- |
| PC                  | Points to instruction           | Drives execution flow                   |
| Instruction Memory  | Stores program                  | Supplies instructions                   |
| Decoder             | Extracts fields                 | Helps regfile & ALU know what to do     |
| Immediate Generator | Builds offsets & constants      | Needed for arithmetic, memory, branches |
| Register File       | Holds 32 registers              | Operands + results                      |
| Control Unit        | High-level command              | Tells datapath which actions to take    |
| ALU Control         | Low-level ALU command           | Selects exact ALU operation             |
| ALU                 | Performs math, logic            | Core computation engine                 |
| Data Memory         | Stores runtime data             | Loads & stores                          |
| Writeback MUX       | Puts result back into registers | Completes instruction                   |


