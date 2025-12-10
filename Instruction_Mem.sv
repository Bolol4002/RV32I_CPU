module Instruction_Mem(
    input clk, reset;
    input [31:0] read_addr;
    output reg [31:0] instruction_out;
    reg [31:0] I_Mem[63:0];
    );

    always @(posedge clk)
    endmodule