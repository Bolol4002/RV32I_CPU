# Notes

---
## WEEK 1
### What is a processor??
- CPU - for general purpose computation
- GPU - graphical processing unit, mostly develop to render stuff like videos
- TPU - tensor processing unit, neural network math best at ai training and inference
- NPU - neural processing unit, built for ai locally on device.



### Architecture vs Microarchitecture, CISC vs RISC
Architecture (Instruction Set Architecture, ISA)
The formal specification that defines what a processor does from the programmer’s point of view, including instructions, registers, data types, memory model, and exception behavior.

Microarchitecture
The internal hardware organization that implements an ISA, describing how instructions are executed using pipelines, execution units, caches, and control logic.
Like for example x86 is used by intel and amd they bring their own flavours.

CISC (Complex Instruction Set Computer)
A processor architecture philosophy characterized by a large and complex instruction set, often with variable-length instructions and single instructions that can perform multiple low-level operations.

RISC (Reduced Instruction Set Computer)
A processor architecture philosophy characterized by a small, simple instruction set with fixed-length instructions, emphasizing single-cycle execution, load–store design, and efficient pipelining.



### High level VS Machine Level language
Compiler - converts high level to machine level language(assembly).
High level - human understandable
Machine level - understood by machines like 8086
    ```
    ADD X1,X2,X3
    ```

---

## WEEK 2
