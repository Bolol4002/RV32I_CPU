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
