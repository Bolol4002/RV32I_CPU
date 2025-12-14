import os
from pathlib import Path
from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "src"
TB_DIR = ROOT / "tb"

SIM = os.getenv("SIM", "verilator")


def run_test(module_name):
    print(f"\n=== Running test for {module_name} ===")

    runner = get_runner(SIM)

    sources = list(SRC_DIR.glob("*.sv"))

    runner.build(
        sources=sources,
        hdl_toplevel=module_name,
        build_dir=TB_DIR / module_name / "sim_build",
        always=True,
    )

    runner.test(
        hdl_toplevel=module_name,
        test_module=f"test_{module_name}",
        test_dir=TB_DIR / module_name,
    )


def test_full_adder():
    run_test("full_adder")


def test_memory():
    run_test("memory")

def test_regfile():
    run_test("regfile")

def test_alu():
    run_test("alu")


if __name__ == "__main__":
    test_full_adder()
    test_memory()
    test_regfile()
    test_alu()
