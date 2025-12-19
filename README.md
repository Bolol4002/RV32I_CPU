
# Notes

---

## WEEK 1

### What is a processor??

* **CPU** – General-purpose computation
* **GPU** – Graphics processing unit, originally developed for rendering videos and images
* **TPU** – Tensor Processing Unit, optimized for neural network math (AI training and inference)
* **NPU** – Neural Processing Unit, built for on-device AI workloads

---

### Architecture vs Microarchitecture, CISC vs RISC

#### Architecture (Instruction Set Architecture – ISA)

The formal specification that defines what a processor does from the programmer’s point of view, including:

* Instructions
* Registers
* Data types
* Memory model
* Exception behavior

#### Microarchitecture

The internal hardware organization that implements an ISA:

* Pipelines
* Execution units
* Caches
* Control logic

Example: x86 is the ISA, but Intel and AMD implement different microarchitectures.

---

#### CISC (Complex Instruction Set Computer)

* Large and complex instruction set
* Variable-length instructions
* Single instruction can perform multiple low-level operations

#### RISC (Reduced Instruction Set Computer)

* Small, simple instruction set
* Fixed-length instructions
* Load–store architecture
* Efficient pipelining
* Emphasis on simple, fast execution

---

### High-level vs Machine-level language

* **Compiler** – Converts high-level language to machine-level (assembly)
* **High-level language** – Human-readable
* **Machine-level language** – Understood by hardware

Example:

```asm
ADD X1, X2, X3
```

---

## WEEK 2

### General architecture we are following

*(Diagram reference)*

---

### PC – Program Counter

* Holds the address of the next instruction to execute
* Points to instruction memory

Flow:

* PC → instruction memory
* Instruction fetched

---

### Instruction Register (IR)

In a CPU, an instruction is just a **32-bit number** (in RV32I).

The **Instruction Register (IR)** is a hardware register that holds the current instruction being executed.

Flow:

* PC points to memory
* Instruction memory returns 32 bits
* Those 32 bits are latched into the Instruction Register
* The CPU decodes fields inside those 32 bits

Important:

* The IR is **not smart**
* It does **not understand** instructions
* It only holds bits

---

### Why instruction formats exist

RISC-V instructions are fixed at **32 bits**, but different instructions need different information:

* Some need two registers
* Some need an immediate
* Some need a jump offset
* Some need no destination register

You cannot fit everything into one layout cleanly.

So RISC-V defines **formats (types)**.

Each format is a **contract between software and hardware**.

---

### The common rule (very important)

All RV32I instructions share:

```
[6:0] opcode
```

Decoding flow:

1. Look at opcode
2. Select instruction format
3. Extract fields accordingly

---

## R-Type Instructions (Register–Register)

### What they do

Operations purely between registers.

Examples:

```asm
add x3, x1, x2
sub x5, x6, x7
and, or, slt, etc.
```

Conceptually:

```
rd = f(rs1, rs2)
```

### Bit layout

```
31     25 | 24   20 | 19   15 | 14  12 | 11   7 | 6    0
------------------------------------------------------
 funct7   |  rs2    |  rs1    | funct3 |   rd   | opcode
```

### Field meaning

* `rs1` → first source register
* `rs2` → second source register
* `rd` → destination register
* `funct3 + funct7` → exact operation

Why both `funct3` and `funct7`?

* Opcode space is precious

Example:

* ADD and SUB share:

```
opcode = 0110011
funct3 = 000
```

* Differ only in `funct7`

Hardware reality:

```
ALU control = f(opcode, funct3, funct7)
```

---

## I-Type Instructions (Immediate)

### What they do

Use **one register + constant value**.

Examples:

```asm
addi x3, x1, 10
lw   x5, 0(x2)
jalr x1, x2, 0
```

Conceptually:

```
rd = f(rs1, immediate)
```

### Bit layout

```
31        20 | 19   15 | 14  12 | 11   7 | 6    0
------------------------------------------------
  immediate  |  rs1    | funct3 |   rd   | opcode
```

### Immediate

* 12 bits
* Signed
* Sign-extended to 32 bits

### Why I-type exists

Constants are everywhere:

* Stack offsets
* Array indexing
* Pointer arithmetic
* Loads and stores

Without immediates, register usage would explode.

---

## Addressing Modes (Very Important)

RISC-V is simple by design.

---

### Register addressing

Used by R-type:

```asm
add x3, x1, x2
```

Operands come only from registers.

---

### Immediate addressing

Used by I-type ALU ops:

```asm
addi x3, x1, 12
```

* `x1` → register
* `12` → encoded in instruction

---

### Base + Offset addressing (CRITICAL)

Used by loads and stores:

```asm
lw x5, 8(x2)
```

Meaning:

```
address = x2 + 8
x5 = MEM[address]
```

Hardware flow:

* Read `rs1`
* Sign-extend immediate
* ALU adds them
* Result goes to memory address port

This is why **ADD support in the ALU is mandatory early**.

---

## S-Type Instructions (Stores)

Stores do **not write registers**, so there is no `rd`.

Immediate is split due to space constraints.

### Bit layout

```
31   25 | 24   20 | 19   15 | 14  12 | 11    7 | 6    0
-----------------------------------------------------
 imm[11:5] | rs2 | rs1 | funct3 | imm[4:0] | opcode
```

Concept:

```
MEM[rs1 + imm] = rs2
```

---

## B-Type Instructions (Branches)

Branches:

* Do not write registers
* Use two registers
* Use PC-relative offset

Example:

```asm
beq x1, x2, label
```

Concept:

```c
if (x1 == x2)
    PC = PC + offset
```

Offset details:

* Split across instruction
* Shifted because instructions are word-aligned
* LSB is always 0, so it is not stored

---

## U-Type Instructions (Upper Immediate)

Used for large constants.

Example:

```asm
lui x5, 0x12345
```

Meaning:

```
x5 = 0x12345 << 12
```

Avoids needing full 32-bit immediates everywhere.

---

## J-Type Instructions (Jumps)

Used by `jal`.

Example:

```asm
jal x1, label
```

Concept:

```
x1 = PC + 4
PC = PC + offset
```

Used for:

* Function calls
* Long jumps

---

## Big Picture (Key Insight)

Instruction formats exist because:

* Hardware wants simple, fixed wiring
* Software wants expressiveness

RISC-V balances both.

Your CPU pipeline will have blocks like:

* Instruction Decode
* Immediate Generator
* Register File
* ALU Control

All depend on instruction type.

---

## Final Takeaway (Memorize This)

* **R-type** → reg + reg → ALU
* **I-type** → reg + imm → ALU / load / jalr
* **S-type** → reg + imm → store
* **B-type** → reg vs reg → branch
* **U-type** → large immediate
* **J-type** → jump + link

---

If you want, next I can:

* Map these formats directly to your **Verilog decode logic**
* Or walk through **one instruction bit-by-bit through the datapath**
* Or help you turn this into **exam-ready condensed notes**

Your move.
