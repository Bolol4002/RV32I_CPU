# RV32I_CPU

---



## **MODULE 1 — regfile.sv**

**Role:** 32 × 32-bit register file.  
**Must support:**

- 2 read ports
    
- 1 write port
    
- x0 always = 0
    

**Inputs:** clk, wen, waddr, wdata, raddr1, raddr2  
**Outputs:** rdata1, rdata2

**When this passes:**

- You can read/write registers except x0.
    
- You simulate a tiny TB that writes and reads values.
    


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

**Role:** Performs RV32I ALU operations.  
**Ops:** add, sub, and, or, xor, slt, sltu, sll, srl, sra

**Inputs:** a, b, alu_op (4–5 bits)  
**Output:** result

**When this passes:**

- You feed values through a simple TB and verify all ops.
    
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

**Role:** Extract immediates from instruction formats (I, S, B, U, J).

**Inputs:** instr  
**Outputs:** imm

**When this passes:**

- imm for addi, lw, sw, beq, jal, etc. all correct.
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

**Role:** Decode RV32I instruction and generate control signals.

**Inputs:** instr  
**Outputs:**

- alu_src
- mem_read
- mem_write
- reg_write
- mem_to_reg
- branch
- jump
- alu_op
**When this passes:**
- All instruction types create correct control signals.

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

### data_memory

**Role:** Simple byte-addressable memory.  
Supports lw/sw fully.

**When this passes:**

- You can load/store values using a small test.
    

---

## **MODULE 6 — pc.sv**

**Role:** Program counter register.

**Inputs:** clk, reset, next_pc  
**Output:** pc_out

**When this passes:**

- PC increments properly.
    
- PC jumps/branches correctly.
    

---

## **MODULE 7 — cpu_single.sv (top of single-cycle core)**

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

---

