# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_simple_cpu(dut):
    dut._log.info("Start Simple 4-bit CPU Test")

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Initial setup
    dut._log.info("Initial Setup")
    dut.ena.value = 1
    dut.ui_in.value = 0  # All control signals low
    dut.uio_in.value = 0
    dut.rst_n.value = 0  # Assert reset
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1  # Release reset
    await ClockCycles(dut.clk, 2)

    # Helper function to set CPU control inputs
    async def set_cpu_inputs(run, reset_cpu, prog_sel):
        # ui_in[0] = RUN, ui_in[1] = RESET_CPU, ui_in[3:2] = PROGRAM_SELECT
        dut.ui_in.value = (prog_sel << 2) | (reset_cpu << 1) | run
        await ClockCycles(dut.clk, 1)

    # Helper function to read CPU outputs
    def read_cpu_outputs():
        data_out = dut.uo_out.value & 0xF  # uo_out[3:0]
        output_valid = (dut.uo_out.value >> 4) & 1  # uo_out[4]
        halted = (dut.uo_out.value >> 5) & 1  # uo_out[5]
        pc_upper = (dut.uo_out.value >> 6) & 3  # uo_out[7:6]
        pc_lower = dut.uio_out.value & 3  # uio_out[1:0]
        pc = (pc_upper << 2) | pc_lower
        return data_out, output_valid, halted, pc

    # Test 1: CPU Reset Test
    dut._log.info("Test 1: CPU Reset")
    await set_cpu_inputs(run=0, reset_cpu=1, prog_sel=0)
    await ClockCycles(dut.clk, 2)
    data_out, output_valid, halted, pc = read_cpu_outputs()
    dut._log.info(
        f"After reset: PC={pc}, DATA={data_out}, VALID={output_valid}, HALTED={halted}"
    )
    assert pc == 0, f"Expected PC=0 after reset, got {pc}"
    assert halted == 0, f"Expected HALTED=0 after reset, got {halted}"
    assert output_valid == 0, f"Expected OUTPUT_VALID=0 after reset, got {output_valid}"

    # Test 2: Run CPU Program (Counter Program)
    dut._log.info("Test 2: Run CPU Program")
    await set_cpu_inputs(run=1, reset_cpu=0, prog_sel=0)

    expected_sequence = [
        # PC=0: LOAD A, 1 -> A=1
        {"pc": 1, "data": 1, "valid": 0},
        # PC=1: OUTPUT A -> Output A=1, set valid flag
        {"pc": 2, "data": 1, "valid": 1},
        # PC=2: ADD A, 1 -> A=2
        {"pc": 3, "data": 2, "valid": 0},
        # PC=3: NOP
        {"pc": 4, "data": 2, "valid": 0},
        # PC=4-14: NOPs (continue incrementing A and outputting)
        {"pc": 5, "data": 2, "valid": 0},
        {"pc": 6, "data": 2, "valid": 0},
        {"pc": 7, "data": 2, "valid": 0},
        {"pc": 8, "data": 2, "valid": 0},
        {"pc": 9, "data": 2, "valid": 0},
        {"pc": 10, "data": 2, "valid": 0},
        {"pc": 11, "data": 2, "valid": 0},
        {"pc": 12, "data": 2, "valid": 0},
        {"pc": 13, "data": 2, "valid": 0},
        {"pc": 14, "data": 2, "valid": 0},
        {"pc": 15, "data": 2, "valid": 0},
        # PC=15: HALT -> should halt execution
        {"pc": 15, "data": 2, "valid": 0, "halted": 1},
    ]

    for i, expected in enumerate(expected_sequence):
        await ClockCycles(dut.clk, 1)
        data_out, output_valid, halted, pc = read_cpu_outputs()
        dut._log.info(
            f"Step {i}: PC={pc}, DATA={data_out}, VALID={output_valid}, HALTED={halted}"
        )

        assert pc == expected["pc"], f"Step {i}: Expected PC={expected['pc']}, got {pc}"
        assert data_out == expected["data"], (
            f"Step {i}: Expected DATA={expected['data']}, got {data_out}"
        )
        assert output_valid == expected["valid"], (
            f"Step {i}: Expected VALID={expected['valid']}, got {output_valid}"
        )

        if "halted" in expected:
            assert halted == expected["halted"], (
                f"Step {i}: Expected HALTED={expected['halted']}, got {halted}"
            )
            if halted:
                break

    # Test 3: Verify CPU stays halted
    dut._log.info("Test 3: Verify CPU stays halted")
    for _ in range(5):
        await ClockCycles(dut.clk, 1)
        data_out, output_valid, halted, pc = read_cpu_outputs()
        assert halted == 1, f"Expected CPU to stay halted, got HALTED={halted}"
        assert pc == 15, f"Expected PC to stay at 15 when halted, got {pc}"

    # Test 4: CPU Reset while running
    dut._log.info("Test 4: CPU Reset while halted")
    await set_cpu_inputs(run=1, reset_cpu=1, prog_sel=0)
    await ClockCycles(dut.clk, 2)
    data_out, output_valid, halted, pc = read_cpu_outputs()
    dut._log.info(
        f"After reset while running: PC={pc}, DATA={data_out}, VALID={output_valid}, HALTED={halted}"
    )
    assert pc == 0, f"Expected PC=0 after reset while running, got {pc}"
    assert halted == 0, f"Expected HALTED=0 after reset while running, got {halted}"

    # Test 5: Verify bidirectional pin configuration
    dut._log.info("Test 5: Check bidirectional pin configuration")
    uio_oe = dut.uio_oe.value
    # uio[1:0] should be outputs (enabled), uio[7:2] should be inputs (disabled)
    expected_oe = 0b00000011  # Only bits 1:0 are outputs
    assert uio_oe == expected_oe, (
        f"Expected uio_oe=0b{expected_oe:08b}, got 0b{uio_oe:08b}"
    )

    # Test 6: Verify unused outputs are properly assigned
    dut._log.info("Test 6: Check unused bidirectional outputs")
    uio_out = dut.uio_out.value
    unused_uio_bits = (uio_out >> 2) & 0x3F  # Bits [7:2] should be 0
    assert unused_uio_bits == 0, (
        f"Expected unused uio_out bits to be 0, got {unused_uio_bits}"
    )

    dut._log.info("✅ All Simple 4-bit CPU tests passed!")
