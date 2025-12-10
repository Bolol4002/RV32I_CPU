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