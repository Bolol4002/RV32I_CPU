import cocotb
from cocotb.triggers import Timer

@cocotb.test()
async def test_full_adder(dut):
    """Test all input combinations of a full adder"""

    # Loop over all 2^3 = 8 combinations
    for a in [0, 1]:
        for b in [0, 1]:
            for cin in [0, 1]:
                dut.a.value = a
                dut.b.value = b
                dut.cin.value = cin

                await Timer(1, units="ns")

                expected = a + b + cin
                expected_sum = expected & 1
                expected_cout = (expected >> 1) & 1

                assert dut.sum.value == expected_sum, f"sum mismatch: {a}+{b}+{cin}"
                assert dut.cout.value == expected_cout, f"cout mismatch: {a}+{b}+{cin}"
