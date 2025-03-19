# Qt-Based PID Simulator

## Overview
This Qt-based PID Simulator provides a graphical representation of a Proportional-Integral-Derivative (PID) control system. The simulator models a heating system with adjustable parameters to observe and analyze temperature regulation behavior.

 ![alttext](https://github.com/cpw1203/PID_SIM/blob/main/PNG_OF_PID.png?raw=true)

 
## Features
- **Real-time Graphing**: Displays target and actual temperature changes over time.
- **PID Control**: Adjustable proportional (Kp), integral (Ki), and derivative (Kd) gains.
- **Simulation of Heating System**: Models heat dissipation and control response.
- **User Input Interface**: Interactive GUI for setting PID parameters and simulation conditions.
- **Data Logging**: Outputs simulation data for further analysis.

## Installation
### Prerequisites
Ensure you have the following installed:
- Qt (with QtCharts module)
- C++ compiler supporting C++11 or later
- CMake (if building from source)

### Build Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/cpw1203/PID_SIM.git
   cd qt-pid-simulator
   ```
2. Configure and build the project:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```
3. Run the simulator:
   ```bash
   ./pid_simulator
   ```

## Usage
1. Start the application.
2. Enter desired PID parameters (Kp, Ki, Kd), target temperature, and other settings.
3. Run the simulation to visualize the system's response.
4. Adjust parameters dynamically to fine-tune the control behavior.

## Code Structure
- **Main.cxx**: Entry point for the application, initializes the GUI.
- **Main.hxx**: Defines simulation structures and logic.
- **SimData**: Stores simulation settings and data.
- **Heater**: Models the heating element and temperature changes.
- **PID**: Implements the PID control algorithm.
- **GUI Components**: Interactive widgets for user input.

## Future Enhancements
- Implement additional control schemes (e.g., adaptive PID, fuzzy logic control).
- Support for data export in CSV format.
- Integration with external sensors for real-world PID tuning.

## License
This project is licensed under the MIT License.

## Acknowledgments
Special thanks to the Qt community for providing a robust framework for UI and data visualization.

 
