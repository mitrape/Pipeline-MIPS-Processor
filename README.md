# MIPS Pipeline CPU

Welcome to the **MIPS Pipeline CPU** project in Verilog! This repository contains a simplified 5‑stage pipelined processor inspired by a classic RISC architecture, including hazard detection, data forwarding, and flush logic. Explore the code, run the testbenches, and watch your MIPS CPU come to life!

---

## Table of Contents
- [Introduction](#introduction)
- [Project Overview & Features](#project-overview--features)
- [Architecture](#architecture)
- [Modules](#modules)
- [Pipeline Stages](#pipeline-stages)
- [Walkthrough Example](#walkthrough-example)
- [Challenges](#challenges)
- [Getting Started](#getting-started)
- [Conclusion](#conclusion)

---

## Introduction

This project implements a simplified **MIPS 5‑stage pipelined CPU** using Verilog. It covers the classic pipeline stages—**Instruction Fetch (IF)**, **Instruction Decode (ID)**, **Execute (EX)**, **Memory Access (MEM)**, and **Write Back (WB)**—and includes essential mechanisms like **hazard detection**, **data forwarding**, and **pipeline flushing**. The goal is to simulate a functional and efficient instruction pipeline, similar to those used in real‑world processors.

## Project Overview & Features

A fully simulated **5‑stage pipelined CPU** with dedicated pipeline registers between stages to allow multiple instructions to be processed simultaneously.
Key features include:

- **Hazard Detection Unit** to prevent data inconsistencies
- **Forwarding Unit** to resolve data hazards efficiently
- **Flush mechanism** to handle control hazards from branches and jumps
- **Two self‑checking testbenches**: one for basic instruction execution, and one focused on flushes, stalls, and data hazards

## Architecture

At a high level, the datapath connects the five stages via pipeline registers (**IF/ID**, **ID/EX**, **EX/MEM**, **MEM/WB**). Control and data signals flow forward; hazard/forwarding logic may feed back to earlier stages to stall, flush, or forward operands when required.

## Modules

- **InstructionMemory** – Stores program instructions; given the current **PC**, outputs the corresponding instruction to the pipeline.
- **DataMemory** – Provides read/write access during **MEM**; enables `lw`/`sw` behavior.
- **RegisterFile** – 32 registers; supports reading two source registers and writing one destination.
- **MainControl** – Decodes the **opcode** and generates control signals for the pipeline.
- **ALU** – Performs arithmetic/logic operations (add, sub, AND, OR, etc.) based on control inputs.
- **ALUControl** – Determines the specific ALU operation from `funct` and `ALUOp`.
- **HazardDetectionUnit** – Detects data hazards; can **stall** by freezing PC and pipeline register updates.
- **ForwardingUnit** – Minimizes stalls by forwarding results from **EX/MEM** or **MEM/WB** directly to ALU inputs.
- **FlushUnit** – Clears instruction/control signals at **IF/ID** on mispredicted branch/jump to prevent incorrect propagation.
- **Pipeline Registers** – **IF/ID**, **ID/EX**, **EX/MEM**, **MEM/WB** hold intermediate data/control between stages.
- **PipelineCPU (top)** – Top‑level that wires together all modules.
- **ProgramCounter (PC)** – Holds current instruction address; normally increments, can jump/branch, or hold during a stall.

## Pipeline Stages

- **Instruction Fetch (IF)** – Read the instruction at the current **PC** and send it forward along with the next PC.
- **Instruction Decode (ID)** – Decode the instruction, read registers, handle immediates/branch prep, and produce control signals.
- **Execute (EX)** – Perform ALU operations or compute memory addresses; forwarding may supply operands to avoid hazards.
- **Memory Access (MEM)** – Perform data memory read/write for `lw`/`sw`; otherwise often a pass‑through.
- **Write Back (WB)** – Write ALU or memory results back to the register file.

## Walkthrough Example

Consider the instruction pair:

```mips
add $t1, $t2, $t3
sub $t4, $t1, $t5
```

**Clock 1**: `add` enters **IF**.  
**Clock 2**: `add` enters **ID** (control generated; `$t2`, `$t3` read). `sub` enters **IF**.  
**Clock 3**: `add` enters **EX** (ALU computes `$t2 + $t3`). `sub` enters **ID** (reads `$t1`, `$t5`, but `$t1` is not yet updated).  
**Clock 4**: `add` enters **MEM**. `sub` enters **EX** and needs `$t1` which **add** just produced in **EX**. The **Forwarding Unit** forwards the ALU result directly for `sub`.  
**Clock 5**: `add` enters **WB** (writes `$t1`). `sub` enters **MEM** (passes result forward).  
**Clock 6**: `sub` enters **WB** (writes `$t4`).

## Challenges

Compared to a single‑cycle design, the pipelined CPU requires careful handling of **parallelism** and **data/control hazards**. Implementing **HazardDetectionUnit**, **ForwardingUnit**, and **FlushUnit**, plus maintaining intermediate state in pipeline registers, are the trickiest parts. While the steps look straightforward on paper, translating them into modular Verilog and integrating them at the top level is non‑trivial.

## Getting Started

1. Open an online Verilog compiler (for example: https://www.jdoodle.com/execute-verilog-online).
2. Copy the contents of `pipeline_final.v` and paste it into the compiler.
3. Choose which testbench to run:
   - For **basic instructions**, **comment out** the second testbench.
   - For **flush, stalls, and data hazards**, **comment out** the first testbench.
4. Click **Execute**.
5. Wait for the simulation to complete and view the output.
6. The testbenches are **self‑checking**; a success message indicates all instructions worked as expected.

## Conclusion

This project demonstrates the design and simulation of a 5‑stage pipelined MIPS processor in Verilog, complete with hazard detection, forwarding, and flushing. Through modular design and comprehensive testing, it offers a practical understanding of how CPUs manage instruction‑level parallelism and address pipeline challenges—serving as a solid foundation for deeper study of advanced processor architectures.
