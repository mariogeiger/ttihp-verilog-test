# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_simple_cpu(dut):
    dut._log.info("Start Simple 4-bit CPU with Conditional Jumps Test")

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

    # Helper function to read CPU outputs and debug info
    def read_cpu_outputs():
        data_out = dut.uo_out.value & 0xF  # uo_out[3:0]
        output_valid = (dut.uo_out.value >> 4) & 1  # uo_out[4]
        halted = (dut.uo_out.value >> 5) & 1  # uo_out[5]
        pc_upper = (dut.uo_out.value >> 6) & 3  # uo_out[7:6]
        pc_lower = dut.uio_out.value & 3  # uio_out[1:0]
        pc = (pc_upper << 2) | pc_lower
        return data_out, output_valid, halted, pc

    # Helper function to read debug information
    def read_debug_info():
        debug_opcode = (dut.uio_out.value >> 2) & 3  # uio_out[3:2]
        debug_operand = (dut.uio_out.value >> 4) & 0xF  # uio_out[7:4]
        return debug_opcode, debug_operand

    # Enhanced debug logging function
    def log_cpu_state(step_name, data_out, output_valid, halted, pc):
        debug_opcode, debug_operand = read_debug_info()
        opcode_names = {0: "NOP", 1: "LOAD", 2: "ADD", 3: "EXT"}
        opcode_name = opcode_names.get(debug_opcode, "UNK")

        # Decode the current instruction for better understanding
        instruction_word = (debug_opcode << 4) | debug_operand
        instruction_description = decode_instruction(debug_opcode, debug_operand)

        dut._log.info(f"{step_name}:")
        dut._log.info(
            f"  STATE: PC={pc}, DATA={data_out}, VALID={output_valid}, HALTED={halted}"
        )
        dut._log.info(f"  INSTR: 0x{instruction_word:02x} = {instruction_description}")
        dut._log.info(
            f"  DEBUG: opcode={debug_opcode}({opcode_name}), operand=0x{debug_operand:x}"
        )
        dut._log.info(
            f"  RAW: uo_out=0x{int(dut.uo_out.value):02x}, uio_out=0x{int(dut.uio_out.value):02x}"
        )
        dut._log.info("")

    # Instruction decoder for debug output
    def decode_instruction(opcode, operand):
        if opcode == 0:
            return "NOP"
        elif opcode == 1:
            return f"LOAD A, {operand}"
        elif opcode == 2:
            return f"ADD A, {operand}"
        elif opcode == 3:
            full_instr = (opcode << 4) | operand
            if full_instr == 0x30:
                return "MOVE A, B"
            elif full_instr == 0x31:
                return "MOVE B, A"
            elif full_instr == 0x32:
                return "OUTPUT A"
            elif full_instr == 0x33:
                return "HALT"
            elif full_instr == 0x34:
                return "JZ 4"
            elif full_instr == 0x35:
                return "JNZ 1"
            elif full_instr == 0x38:
                return "JZ 8"
            elif full_instr == 0x3A:
                return "JNZ 10"
            else:
                return f"UNKNOWN_EXT(0x{full_instr:02x})"
        else:
            return f"INVALID_OPCODE({opcode})"

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

    # Test 2: Run CPU Program with Conditional Jumps
    dut._log.info("Test 2: Run CPU Program with Conditional Jumps")

    # First make sure CPU is not in reset
    await set_cpu_inputs(run=0, reset_cpu=0, prog_sel=0)
    await ClockCycles(dut.clk, 2)

    # Now start the CPU
    await set_cpu_inputs(run=1, reset_cpu=0, prog_sel=0)
    await ClockCycles(dut.clk, 2)  # Give CPU time to start

    # Check initial state after starting
    data_out, output_valid, halted, pc = read_cpu_outputs()
    log_cpu_state("Initial CPU State", data_out, output_valid, halted, pc)
    dut._log.info(
        f"Input signals: ui_in=0x{int(dut.ui_in.value):02x}, rst_n={dut.rst_n.value}, ena={dut.ena.value}"
    )
    dut._log.info("")

    expected_sequence = [
        # After PC=0: LOAD A, 0 -> A=0, PC will become 1 next cycle
        {"pc": 1, "data": 0, "valid": 0},
        # After PC=1: JZ 4 -> Jump to 4 (since A=0), PC becomes 4
        {"pc": 4, "data": 0, "valid": 0},
        # After PC=4: LOAD A, 5 -> A=5, PC becomes 5
        {"pc": 5, "data": 5, "valid": 0},
        # After PC=5: OUTPUT A -> Output A=5, set valid flag, PC becomes 6
        {"pc": 6, "data": 5, "valid": 1},
        # After PC=6: JZ 8 -> Should NOT jump (A=5 != 0), PC becomes 7
        {"pc": 7, "data": 5, "valid": 0},
        # After PC=7: JNZ 10 -> Jump to 10 (since A=5 != 0), PC becomes 10
        {"pc": 10, "data": 5, "valid": 0},
        # After PC=10: LOAD A, 0 -> A=0, PC becomes 11
        {"pc": 11, "data": 0, "valid": 0},
        # After PC=11: OUTPUT A -> Output A=0, set valid flag, PC becomes 12
        {"pc": 12, "data": 0, "valid": 1},
        # After PC=12: HALT -> should halt execution, PC stays at 12
        {"pc": 12, "data": 0, "valid": 0, "halted": 1},
    ]

    for i, expected in enumerate(expected_sequence):
        await ClockCycles(dut.clk, 2)  # Give more time for PC to update
        data_out, output_valid, halted, pc = read_cpu_outputs()
        log_cpu_state(f"Step {i}", data_out, output_valid, halted, pc)

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

    dut._log.info("✅ All Simple 4-bit CPU with Conditional Jumps tests passed!")
