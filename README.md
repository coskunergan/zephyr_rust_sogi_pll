# SOGI-PLL Implementation in Rust for Zephyr RTOS

## Overview
This project implements a **Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL)** algorithm using the Rust programming language, targeting the Zephyr Real-Time Operating System (RTOS). The SOGI-PLL is designed to synchronize with an input signal, typically for applications like grid synchronization, power electronics, or signal processing, by estimating the phase and frequency of the input signal. The implementation leverages Rust's safety guarantees and Zephyr's real-time capabilities to ensure reliable and efficient performance on embedded systems.

The code is written in a `no_std` environment, making it suitable for resource-constrained embedded devices. It uses the `embassy` framework for task management and asynchronous execution, and it supports hardware interfaces for ADC (Analog-to-Digital Converter), DAC (Digital-to-Analog Converter), and a display for real-time monitoring.


![Prj Demo](img/demo.gif)


## Features
- **SOGI-PLL Algorithm**: Implements a robust SOGI-PLL for phase and frequency estimation of a 50 Hz nominal input signal, with a sampling frequency of 2500 Hz.
- **Fixed-Point Arithmetic**: Uses Q15 fixed-point arithmetic for efficient computation on embedded systems without floating-point hardware.
- **PID Controller**: Integrates a PID controller to stabilize the PLL's frequency tracking, with configurable gains (`KP`, `KI`, `KC`).
- **Automatic Offset Calibration**: Dynamically adjusts the input signal's offset to center it for accurate processing.
- **Real-Time Monitoring**: Displays PLL status (lock state, frequency, phase, and processing time) on a connected display, updated every 100 ms.
- **Asynchronous Execution**: Utilizes the `embassy` framework for task scheduling and interrupt-driven ADC sampling.
- **Hardware Integration**:
  - ADC: Reads input signal asynchronously with a configurable sampling period.
  - DAC: Outputs a synchronized sinusoidal signal based on the estimated phase.
  - Display: Shows real-time metrics for debugging and monitoring.
- **Logging**: Outputs detailed state information (e.g., offset, omega, theta, error) via a logging interface for debugging.
- **Modular Design**: Separates ADC, DAC, and display I/O into dedicated modules (`adc_io`, `dac_io`, `display_io`) for maintainability.

## Prerequisites
To build and run this project, you need the following:
- **Rust Toolchain**: Install the Rust toolchain with support for `no_std` environments. Use `rustup` to target the appropriate architecture (e.g., `thumbv7em-none-eabihf` for Cortex-M).
- **Zephyr RTOS**: Set up the Zephyr development environment, including the Zephyr SDK and dependencies. Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).
- **Embassy Framework**: Ensure the `embassy` crate is included in your project dependencies for task management and time handling.
- **Hardware**:
  - A microcontroller supported by Zephyr (e.g., STM32, nRF52) with ADC, DAC, and display interfaces.
  - Connections for ADC input (e.g., a 50 Hz sinusoidal signal), DAC output, and a compatible display.
- **Build Tools**: Install `west` (Zephyr's build tool) and other dependencies as outlined in the Zephyr documentation.

## Project Structure
- **`lib.rs`**: The main Rust source file containing the SOGI-PLL algorithm, fixed-point math utilities, PID controller, and task management.
- **`adc_io.rs`**: Module for ADC configuration and asynchronous sampling.
- **`dac_io.rs`**: Module for DAC configuration and output.
- **`display_io.rs`**: Module for display initialization and updates.
- **`usage.rs`**: Utility module for performance measurement and logging.
- **Dependencies**:
  - `alloc`: For dynamic memory allocation in a `no_std` environment.
  - `embassy_executor`: For task spawning and scheduling.
  - `embassy_time`: For precise timing and delays.
  - `spin`: For thread-safe static state management (`RwLock`).
  - `static_cell`: For initializing static variables safely.

## Building and Running
1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Set Up Zephyr Environment**:
   - Initialize the Zephyr workspace:
     ```bash
     west init -l .
     west update
     ```
   - Ensure your board is supported by Zephyr. Update the `west.yml` or project configuration if needed.

3. **Configure the Project**:
   - Create a Zephyr application directory and link this Rust code as a module.
   - Update the `CMakeLists.txt` to include the Rust code (use `zephyr_rust` or similar integration).
   - Specify your target board in the build command (e.g., `stm32f4_disco`).

4. **Build the Project**:
   ```bash
   west build -b <board-name>
   ```

5. **Flash the Firmware**:
   ```bash
   west flash
   ```

6. **Monitor Output**:
   - Connect to the microcontroller's serial port (e.g., using `minicom` or `screen`) to view log output.
   - Observe the display for real-time PLL metrics (lock status, frequency, processing time).

## Usage
- **Input Signal**: Provide a sinusoidal input signal (e.g., 50 Hz) to the ADC pin. The PLL will estimate its phase and frequency.
- **Output Signal**: The DAC outputs a synchronized sinusoidal signal with an amplitude scaled to the input signal's range (0–4095).
- **Display**: The display updates every 100 ms, showing:
  - `sPLL`: Lock status (1 if locked, 0 if not).
  - `F`: Estimated frequency in Hz.
  - `T`: Processing time per sample in microseconds.
- **Logging**: Detailed state information is logged, including:
  - `OFFSET_MAX` and `OFFSET_MIN`: Input signal offset bounds.
  - `OMEGA`: Angular frequency (rad/s).
  - `THETA`: Phase angle (degrees).
  - `S1` and `S2`: SOGI state variables.
  - `ERR`: PLL error.
  - `FREQ`: Estimated frequency (Hz).
  - `D_TIME`: Processing duration (ns).

## Configuration
The algorithm's behavior can be tuned by modifying constants in `lib.rs`:
- **Sampling Frequency**: `SAMPLE_FREQ` (default: 2500 Hz).
- **Target Frequency**: `TARGET_FREQ` (default: 50 Hz).
- **PID Gains**: `PID_KP_FLOAT`, `PID_KI_FLOAT`, `PID_KC_FLOAT` (default: 150.0, 15.0, 10.0).
- **SOGI Gain**: `SOGI_K_FLOAT` (default: √2 ≈ 1.414).
- **Offset Step**: `OFFSET_STEP_COEFF` (default: 10,000).
- **Frequency Range**: `PID_FREQ_RANGE` (default: ±15 Hz around 50 Hz).

To modify these, update the constants and rebuild the project.

## Performance
- **Processing Time**: The `duration_ns` field logs the time taken to process each ADC sample, typically in the range of microseconds.
- **Lock Time**: The PLL achieves lock (error < 30 mV in Q15 scale) after approximately `N_SAMPLE` samples (default: 50 samples, or 20 ms at 2500 Hz).
- **Accuracy**: The fixed-point arithmetic ensures minimal computational overhead while maintaining sufficient precision for phase and frequency tracking.

## Limitations
- **Hardware Dependency**: The ADC, DAC, and display modules (`adc_io`, `dac_io`, `display_io`) are placeholders and require implementation specific to your hardware.
- **Fixed-Point Precision**: Q15 arithmetic may introduce quantization errors for very small or large signals. Adjust `Q15_SCALE` if higher precision is needed.
- **Single-Channel ADC**: The current implementation assumes a single ADC channel (index 0). Modify `adc_callback` for multi-channel support.
- **Zephyr-Specific**: The code relies on Zephyr's `embassy` integration, which may require adaptation for other RTOS or bare-metal environments.

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit your changes (`git commit -m "Add feature"`).
4. Push to the branch (`git push origin feature-name`).
5. Open a pull request.

Please include tests and documentation for new features.

## License
This project is licensed under the Apache-2.0 License. See the `lib.rs` header for details.

## Author
Coskun Ergan (<coskunergan@gmail.com>)

## Acknowledgments
- Zephyr RTOS community for providing a robust embedded platform.
- Rust embedded community for `no_std` libraries and tools.
- Embassy framework for simplifying asynchronous programming in Rust.