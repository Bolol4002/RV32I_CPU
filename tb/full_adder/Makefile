TOPLEVEL_LANG = verilog

# Simulator
SIM = verilator

# Location of RTL
VERILOG_SOURCES += $(PWD)/../../src/full_adder.sv

# Top module name
TOPLEVEL = full_adder

# Python testbench (no .py)
MODULE = test_full_adder

# Enable waveform output
VERILATOR_TRACE = 1

include $(shell cocotb-config --makefiles)/Makefile.sim
