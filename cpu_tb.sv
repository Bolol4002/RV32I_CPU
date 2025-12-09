module cpu_tb;

    logic clk;
    logic reset;

    cpu_single dut (
        .clk(clk),
        .reset(reset)
    );

    always #5 clk = ~clk;  // 100MHz clock

    initial begin
        clk = 0;
        reset = 1;
        #20;
        reset = 0;

        // run for some cycles
        #200;

        // Show memory[0] where SW stored result
        $display("MEM[0] = %0d",
            {dut.dmem.dmem[3], dut.dmem.dmem[2], dut.dmem.dmem[1], dut.dmem.dmem[0]}
        );

        $finish;
    end

endmodule
