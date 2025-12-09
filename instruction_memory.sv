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
