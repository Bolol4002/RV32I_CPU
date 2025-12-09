module data_memory #(
    parameter SIZE_BYTES = 1024
    ) (
    input  logic         clk,
    input  logic         mem_read,   // 1 = read
    input  logic         mem_write,  // 1 = write (word)
    input  logic [31:0]  addr,       // byte address
    input  logic [31:0]  wdata,      // write data (word)
    output logic [31:0]  rdata
    );
    // byte-addressable memory
    reg [7:0] dmem [0:SIZE_BYTES-1];

    // combinational read (word-aligned)
    always_comb begin
        if (mem_read) begin
            // little-endian assemble
            rdata = { dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr+0] };
        end else begin
            rdata = 32'd0;
        end
    end

    // synchronous write (store word). For simplicity, assume aligned addresses.
    always_ff @(posedge clk) begin
        if (mem_write) begin
            dmem[addr+0] <= wdata[7:0];
            dmem[addr+1] <= wdata[15:8];
            dmem[addr+2] <= wdata[23:16];
            dmem[addr+3] <= wdata[31:24];
        end
    end
endmodule