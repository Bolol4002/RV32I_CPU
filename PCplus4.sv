module PCplus4(input [31:0]fromPC, output [31:0] nextoPC);
    assign nextoPC = fromPC +4;
endmodule