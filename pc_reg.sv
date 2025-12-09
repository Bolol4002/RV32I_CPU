module pc_reg(
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