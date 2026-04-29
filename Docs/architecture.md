# Enterprise Flight Stack Architecture

## Core Design Principles
This repository adheres strictly to professional aerospace flight stack paradigms. The entire codebase guarantees:
1. **Strong Typing via Interfaces**: All inter-subsystem communication is strictly routed through predefined `Simulink.Bus` definitions mapping to discrete `signal_enums.m`.
2. **Multi-Rate Discrete Execution**: Handled via the `/Runtime` task scheduler. No two sensors or control loops operate synchronously unless mathematically bound. Latency injectors simulate bus contention (SPI/I2C/CAN).
3. **Automated Verification**: Handled via `.github/workflows/`, ensuring regression and Monte Carlo tests are performed continuously.

## Directory Map

### 1. Interfaces & Runtime
- `Interfaces/`: Ensures variables like `altitude` don't collide between metric/imperial. Houses `bus_definitions.m`.
- `Runtime/`: Breaks the illusion of instantaneous software execution. Uses `task_scheduler.m` to simulate RTOS (Real-Time Operating System) behavior.

### 2. Physical & Navigational Layers
- `Guidance/`: Contains Jerk-limited planners and dynamic replanning modules.
- `Control/`: Features LQR matrices and MPC experimental hooks.
- `Plant/`: Contains ground effect, payload shifting, and torque modeling.
- `Sensors/`: Replicates ADC quantization limits and thermal drift profiles.

### 3. FDIR (Fault Detection, Isolation, and Recovery)
- `Estimator/`: Houses the 15-state EKF alongside innovation monitoring logic.
- `Faults/`: Triggers responses based on comm loss, brownouts, and desynchronizations.

### 4. Enterprise Pipeline
- `Scenarios/`: Contains programmatic flight tests (e.g., `aggressive_turn_test.m`).
- `Logs/`: Replays data and exports to ULog format for PX4 compatibility.
- `Validation/`: Computes code coverage and tests stress campaigns.
- `Codegen/`: Enforces strict MISRA-C RAM/ROM budgets prior to ERT compilation.
