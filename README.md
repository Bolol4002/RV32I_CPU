# RV32I_CPU

---


## **Module 1 - program_counter**
```
module program_counter(
    input clk,rst;
    input [31:0] PC_in;
    output reg [31:0] PC_out;
    );
    always @(posedge clk or posedge rst) begin
        if (reset) pc_out <= 32'd0;
        else pc_out <= next_pc;
    end
endmodule
```

---

## **Module 2 - PC+4**
```
module PCplus4(input [31:0]fromPC, output [31:0] nextoPC);
    assign nextoPC = fromPC +4;
endmodule
```

---

## **Module 3 - Instruction Memory**
```

```

---


## **MODULE — regfile.sv**

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

## **MODULE — alu.sv**

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


