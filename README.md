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

# **Stop here and verify before moving forward**

Do not touch pipelining until the single-cycle version passes:

✔ Arithmetic  
✔ Branch taken/not taken  
✔ lw/sw  
✔ jal/jalr  
✔ SVA: x0 never written  
✔ PC always 4-byte aligned

Once these are locked in, we begin PHASE-2 — pipelining.

---

[[phase2_cpu]]
