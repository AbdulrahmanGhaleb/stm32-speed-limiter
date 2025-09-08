# Speed Limiter Firmware (STM32F405RGT6)

This repository contains the firmware for a custom STM32F405-based board designed to act as a **man-in-the-middle speed limiter** between an automotive accelerator pedal and the vehicle’s ECU.

The system reads vehicle speed from the CAN bus, monitors pedal input signals, and dynamically attenuates or cuts off the pedal output once speed thresholds are exceeded.

## Hardware Overview

The custom PCB integrates:
**STM32F405RGT6** microcontroller
**DAC121S101** (x2) for pedal signal reconstruction (SPI)
**TCAN337** CAN transceiver
**USB-C** for CANH/CANL connectivity
**UART headers** for debugging (115200 baud)
**Multiplexer** to route DAC outputs back to the ECU

### Signal Flow

1. **Pedal → STM32 ADC**
   Pedal signals are read through ADC channels (with voltage divider adjustment).

2. **Speed Readout (CAN bus)**
   Vehicle speed is requested and parsed from CAN frames via the TCAN337 transceiver.

3. **Processing (Attenuation Algorithm)**

   * If speed < 30 km/h → pedal signal is passed directly.
   * If 30–50 km/h → pedal signal is progressively attenuated.
   * If ≥ 50 km/h → pedal input is effectively cut off.

4. **STM32 → DAC → Multiplexer → ECU**
   Processed signals are converted back to analog via DACs and routed through the multiplexer to the ECU.



## Functional Overview

* **Normal Operation (< 30 km/h):** Pedal signals are passed through unchanged.
* **Scaling Region (30–50 km/h):** Pedal signals are scaled down linearly.
* **Cut-Off Region (≥ 50 km/h):** Pedal signals are suppressed (ECU sees minimal input).



## Prerequisites

* **Device Under Test (DUT)** flashed with target firmware
* **CAN bus interface adapter** connected to OBD-II
* **USB-to-UART serial adapter**
* **Terminal emulator** configured for **115200 baud, 8N1**



## Test Procedure

### 1. Initialization and Power-Up

* Ensure vehicle ignition is **OFF**.
* Connect the DUT to the vehicle’s OBD-II port via CAN.
* Connect the DUT inline between **pedal** and **ECU**.
* Attach USB-to-UART adapter to the DUT debug header and open terminal (115200).

### 2. System Power-On and Communication Check

* Switch ignition to **ON/RUN** (engine not started).
* Confirm status LED is illuminated (board powered).
* Confirm UART session is active (ready to display CAN messages).

### 3. Engine Start and Baseline Validation

* Start the engine.
* Press pedal while stationary → verify normal engine response.

### 4. Pre-Threshold Functional Test

* Accelerate to just below **30 km/h**.
* Confirm pedal works normally (no scaling yet).

### 5. Algorithm Activation Test (Scaling Region)

* Cross **30 km/h threshold**.
* Gradually accelerate toward 50 km/h.
* Confirm pedal sensitivity decreases progressively with speed.

### 6. Upper Limit Validation (Cut-Off Region)

* Maintain speed **≥ 50 km/h**.
* Confirm pedal input is ignored (engine no longer responds to accelerator).

### 7. Recovery

* Reduce speed below **30 km/h**.
* Confirm pedal functionality returns to normal.

