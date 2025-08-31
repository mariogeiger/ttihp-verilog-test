# design.py - Simple 4-bit CPU implementation for Tiny Tapeout
import os

import yaml
from amaranth import Elaboratable, Module, Signal, Array, Const
from amaranth.back import verilog


class SimpleCPU(Elaboratable):
    """
    A minimal 4-bit CPU with the following features:
    - 4-bit data width
    - 2 general-purpose registers (A, B)
    - 4-bit program counter
    - Enhanced instruction set with 8 instructions including conditional jumps
    - Built-in instruction ROM with 16 instruction slots

    Instruction Set (6-bit instructions: 2-bit opcode + 4-bit operand):
    - 00xxxx: NOP - No operation
    - 01xxxx: LOAD A, immediate - Load 4-bit immediate value into register A
    - 10xxxx: ADD A, immediate - Add 4-bit immediate value to register A
    - 110000: MOVE A, B - Copy register A to register B
    - 110001: MOVE B, A - Copy register B to register A
    - 110010: OUTPUT A - Output register A value
    - 110011: HALT - Stop program execution
    - 110100: JZ 4 - Jump to address 4 if register A == 0
    - 110101: JNZ 1 - Jump to address 1 if register A != 0
    - 111000: JZ 8 - Jump to address 8 if register A == 0
    - 111010: JNZ 10 - Jump to address 10 if register A != 0
    - (More jump targets can be encoded with different bit patterns)
    """

    def __init__(self):
        # External interface
        self.run = Signal()  # Start/run CPU (ui_in[0])
        self.reset_cpu = Signal()  # Reset CPU state (ui_in[1])
        self.program_select = Signal(2)  # Select which program to run (ui_in[3:2])

        # Outputs
        self.output_data = Signal(4)  # Data output (uo_out[3:0])
        self.output_valid = Signal()  # Output data valid flag (uo_out[4])
        self.halted = Signal()  # CPU halted flag (uo_out[5])
        self.pc_out = Signal(
            4
        )  # Program counter for debugging (uo_out[7:6] + uio_out[1:0])

    def elaborate(self, platform):
        m = Module()

        # CPU State
        pc = Signal(4)  # Program counter (4 bits = 16 instructions max)
        reg_a = Signal(4)  # Register A
        reg_b = Signal(4)  # Register B
        halt_flag = Signal()  # Halt execution flag
        output_valid_flag = Signal()  # Output valid flag
        jump_flag = Signal()  # Flag to indicate if a jump occurred this cycle

        # fmt: off
        # Instruction memory - 4 different simple programs
        instruction_mem = Array(
            [
                # Program 0: Conditional jump demonstration
                Const(0b010000, 6),  # 0: LOAD A, 0     - Load 0 into A
                Const(0b110100, 6),  # 1: JZ 4          - Jump to 4 if A == 0 (should jump)
                Const(0b110010, 6),  # 2: OUTPUT A      - Should NOT execute (skipped by jump)
                Const(0b110011, 6),  # 3: HALT          - Should NOT execute (skipped by jump)
                Const(0b010101, 6),  # 4: LOAD A, 5     - Load 5 into A
                Const(0b110010, 6),  # 5: OUTPUT A      - Output 5
                Const(0b111000, 6),  # 6: JZ 8          - Should NOT jump (A=5 != 0)
                Const(0b111010, 6),  # 7: JNZ 10        - Should jump to 10 (A=5 != 0)
                Const(0b110011, 6),  # 8: HALT          - Should NOT execute
                Const(0b110011, 6),  # 9: HALT          - Should NOT execute  
                Const(0b010000, 6),  # 10: LOAD A, 0    - Load 0 into A
                Const(0b110010, 6),  # 11: OUTPUT A     - Output 0
                Const(0b110011, 6),  # 12: HALT         - Final halt
                Const(0b000000, 6),  # 13: NOP
                Const(0b000000, 6),  # 14: NOP
                Const(0b000000, 6),  # 15: NOP
            ]
        )
        # fmt: on
        # Alternative programs could be selected with program_select
        # For now, we'll use the same program but could extend this

        # Current instruction
        current_instruction = Signal(6)
        opcode = Signal(2)
        operand = Signal(4)

        # Fetch instruction
        m.d.comb += [
            current_instruction.eq(instruction_mem[pc]),
            opcode.eq(current_instruction[4:6]),  # Top 2 bits
            operand.eq(current_instruction[0:4]),  # Bottom 4 bits
        ]

        # CPU execution logic
        with m.If(self.reset_cpu):
            # Reset all state
            m.d.sync += [
                pc.eq(0),
                reg_a.eq(0),
                reg_b.eq(0),
                halt_flag.eq(0),
                output_valid_flag.eq(0),
                jump_flag.eq(0),
            ]
        with m.Elif(self.run & ~halt_flag):
            # Clear output valid and jump flags by default
            m.d.sync += [output_valid_flag.eq(0), jump_flag.eq(0)]

            # Execute instruction based on opcode
            with m.Switch(opcode):
                with m.Case(0b00):  # NOP
                    pass  # Do nothing

                with m.Case(0b01):  # LOAD A, immediate
                    m.d.sync += reg_a.eq(operand)

                with m.Case(0b10):  # ADD A, immediate
                    m.d.sync += reg_a.eq(reg_a + operand)

                with m.Case(0b11):  # Extended instructions
                    with m.Switch(
                        current_instruction
                    ):  # Use full 6-bit instruction for extended instructions
                        with m.Case(0b110000):  # MOVE A, B
                            m.d.sync += reg_b.eq(reg_a)
                        with m.Case(0b110001):  # MOVE B, A
                            m.d.sync += reg_a.eq(reg_b)
                        with m.Case(0b110010):  # OUTPUT A
                            m.d.sync += output_valid_flag.eq(1)
                        with m.Case(0b110011):  # HALT
                            m.d.sync += halt_flag.eq(1)
                        with m.Case(0b110100):  # JZ 4
                            with m.If(reg_a == 0):
                                m.d.sync += [
                                    pc.eq(4),  # Jump to address 4
                                    jump_flag.eq(1),
                                ]
                        with m.Case(0b110101):  # JNZ 1
                            with m.If(reg_a != 0):
                                m.d.sync += [
                                    pc.eq(1),  # Jump to address 1
                                    jump_flag.eq(1),
                                ]
                        with m.Case(0b111000):  # JZ 8
                            with m.If(reg_a == 0):
                                m.d.sync += [
                                    pc.eq(8),  # Jump to address 8
                                    jump_flag.eq(1),
                                ]
                        with m.Case(0b111010):  # JNZ 10
                            with m.If(reg_a != 0):
                                m.d.sync += [
                                    pc.eq(10),  # Jump to address 10
                                    jump_flag.eq(1),
                                ]

            # Increment program counter only if no jump occurred
            with m.If(~jump_flag):
                m.d.sync += pc.eq(pc + 1)

        # Connect outputs
        m.d.comb += [
            self.output_data.eq(reg_a),  # Always output register A
            self.output_valid.eq(output_valid_flag),
            self.halted.eq(halt_flag),
            self.pc_out.eq(pc),
        ]

        return m


def get_module_name():
    """Read the top module name from info.yaml"""
    try:
        # Go up one directory to find info.yaml
        info_yaml_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "info.yaml"
        )
        with open(info_yaml_path, "r") as f:
            info = yaml.safe_load(f)
        return info["project"]["top_module"]
    except Exception as e:
        print(f"Warning: Could not read module name from info.yaml: {e}")
        return "tt_um_set_reset_gate"  # fallback


def generate_tiny_tapeout_wrapper(amaranth_verilog, module_name=None):
    """Convert Amaranth-generated Verilog to Tiny Tapeout compatible format"""

    if module_name is None:
        module_name = get_module_name()

    # CPU wrapper template that instantiates the Amaranth CPU core
    wrapper = f"""/*
 * Copyright (c) 2024 Mario Geiger
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Auto-generated from Amaranth design - Simple 4-bit CPU
 */

`default_nettype none

module {module_name} (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Internal signals for the Amaranth-generated CPU core
  wire rst = !rst_n;  // Convert active-low reset to active-high
  
  // CPU output signals
  wire [3:0] cpu_output_data;
  wire cpu_output_valid;
  wire cpu_halted;
  wire [3:0] cpu_pc;
  
  // Instantiate the Amaranth-generated CPU core module
  top cpu_core (
    .clk(clk),
    .rst(rst),
    .run(ui_in[0]),              // RUN signal from ui_in[0]
    .reset_cpu(ui_in[1]),        // CPU reset from ui_in[1]  
    .program_select(ui_in[3:2]), // Program select from ui_in[3:2]
    .output_data(cpu_output_data),    // 4-bit data output
    .output_valid(cpu_output_valid),  // Output valid flag
    .halted(cpu_halted),              // CPU halted flag
    .pc_out(cpu_pc)                   // Program counter output
  );
  
  // Connect CPU outputs to Tiny Tapeout pins
  assign uo_out[3:0] = cpu_output_data;  // Data output on uo_out[3:0]
  assign uo_out[4] = cpu_output_valid;   // Output valid flag on uo_out[4]
  assign uo_out[5] = cpu_halted;         // Halted flag on uo_out[5]
  assign uo_out[7:6] = cpu_pc[3:2];      // Upper 2 bits of PC on uo_out[7:6]
  
  // Use bidirectional pins for additional PC bits  
  assign uio_out[1:0] = cpu_pc[1:0];     // Lower 2 bits of PC on uio_out[1:0]
  assign uio_out[7:2] = 6'b0;            // Unused bidirectional outputs
  assign uio_oe[1:0] = 2'b11;            // Enable PC output on uio[1:0]
  assign uio_oe[7:2] = 6'b0;             // Other bidirectional pins as inputs

  // List all unused inputs to prevent warnings  
  wire _unused = &{{ena, ui_in[7:4], uio_in, 1'b0}};

endmodule

// Amaranth-generated CPU core module
{amaranth_verilog}
"""

    return wrapper


if __name__ == "__main__":
    # Generate the Amaranth CPU core module
    cpu = SimpleCPU()
    amaranth_verilog = verilog.convert(
        cpu,
        ports=[
            cpu.run,
            cpu.reset_cpu,
            cpu.program_select,
            cpu.output_data,
            cpu.output_valid,
            cpu.halted,
            cpu.pc_out,
        ],
    )

    # Generate the complete Tiny Tapeout wrapper (module name read from info.yaml)
    complete_project = generate_tiny_tapeout_wrapper(amaranth_verilog)

    # Write to project.v (same directory as this script)
    with open("project.v", "w") as f:
        f.write(complete_project)

    print(
        f"✅ Generated project.v from Amaranth Simple 4-bit CPU design! Module: {get_module_name()}"
    )
