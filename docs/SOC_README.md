# State of Charge (SoC) Calculator Module

## Overview

The State of Charge (SoC) Calculator is a critical component of the Battery Management System (BMS) for the 100KW/200KWH BESS using LFP battery modules. This module provides accurate estimation of battery state of charge using multiple algorithms and techniques specifically optimized for LFP (Lithium Iron Phosphate) battery chemistry.

## Key Features

1. **Multiple SoC Calculation Methods**:
   - **Coulomb Counting**: Integrates current flow over time to track charge movement
   - **OCV-based**: Uses Open Circuit Voltage to estimate SoC from lookup tables
   - **Kalman Filter**: Advanced fusion algorithm that combines predictions with measurements
   - **Hybrid Method**: Combines benefits of multiple approaches for optimal accuracy

2. **LFP Battery Optimizations**:
   - OCV-SoC lookup table specifically calibrated for LFP battery chemistry
   - Handles LFP's characteristically flat voltage curve in mid-SoC ranges
   - Configured for 48V module architecture with 16KWH capacity

3. **Temperature Compensation**:
   - Dynamically adjusts capacity estimates based on operating temperature
   - Comprehensive temperature lookup table from -20°C to 60°C
   - Ensures accuracy across wide operating temperature range

4. **Kalman Filter Implementation**:
   - State estimation algorithm for optimal data fusion
   - Configurable process and measurement noise parameters
   - Dynamically adjusts to changing battery conditions
   - Self-correcting algorithm that improves over operating time

5. **Error Estimation**:
   - Provides quantitative SoC estimation error values
   - Adjusts error estimates based on rest periods and current load
   - Enables system to make informed decisions based on confidence levels

6. **System-Level SoC Calculation**:
   - Calculates both individual module and overall system SoC
   - Weighted averaging based on module capacity
   - Ensures accurate representation of total system energy status

## Implementation Details

### Architecture

The SoC calculator is implemented with a modular architecture consisting of:

- Core calculation engine supporting multiple estimation methods
- Per-module data tracking for up to 16 battery modules
- Thread-safe design with mutex protection for shared data
- FreeRTOS compatible task structure

### Technical Features

- **Thread Safety**: All functions are protected with semaphores for concurrent access
- **Error Handling**: Comprehensive validation and error reporting
- **Logging**: Detailed logging via ESP-IDF's logging system
- **Memory Efficiency**: Optimized data structures for minimal RAM usage
- **Configurability**: Adjustable parameters for different battery configurations

### Core Algorithms

#### Coulomb Counting
Tracks charge flow by integrating current over time:
```
charge_delta = current * (time_delta / 3600.0)
accumulated_charge += charge_delta
soc = (accumulated_charge / nominal_capacity) * 100.0
```

#### OCV-Based Estimation
Uses lookup tables and interpolation to map voltage to SoC:
```
if (battery at rest) {
    soc = interpolate_from_ocv_table(voltage)
}
```

#### Kalman Filter
Combines prediction with measurement for optimal estimation:
```
// Prediction step
kalman_soc += predicted_change
kalman_p += process_noise

// Update step when measurement available
kalman_gain = kalman_p / (kalman_p + measurement_noise)
kalman_soc += kalman_gain * (measured_soc - kalman_soc)
kalman_p = (1 - kalman_gain) * kalman_p
```

### Temperature Compensation

Temperature significantly affects available capacity in LFP batteries. The module implements temperature compensation with the following characteristics:

- At 25°C: 100% of nominal capacity available (optimal)
- At 0°C: Approximately 90% of nominal capacity available
- At -20°C: Approximately 70% of nominal capacity available
- At 60°C: Approximately 90% of nominal capacity available

## Integration Guide

### Initialization

Initialize the module at system startup:

```c
esp_err_t result = soc_calculator_init(SOC_METHOD_HYBRID);
if (result != ESP_OK) {
    // Handle initialization error
}
```

### Regular Updates

Update coulomb counting whenever current measurements are taken:

```c
// Call this every time a new current measurement is available
soc_calculator_update_coulomb_counting(module_id, current, time_delta);
```

### Calculating SoC

Calculate module SoC when needed:

```c
float module_soc = 0.0;
esp_err_t result = soc_calculator_calculate_module_soc(
                        module_id, &module_data, &module_soc);
if (result == ESP_OK) {
    // Use the calculated SoC
}
```

Calculate system-wide SoC:

```c
float system_soc = 0.0;
esp_err_t result = soc_calculator_calculate_system_soc(
                        all_module_data, module_count, &system_soc);
if (result == ESP_OK) {
    // Use the calculated system SoC
}
```

### Calibration

Periodic calibration improves accuracy:

```c
// Call when battery has been at rest (low current) for sufficient time
soc_calculator_calibrate_from_ocv(module_id, rest_voltage);
```

## Advanced Usage

### Kalman Filter Tuning

Tune the Kalman filter for your specific battery behavior:

```c
// Adjust process noise (prediction uncertainty) and 
// measurement noise (OCV reading uncertainty)
soc_calculator_set_kalman_params(0.005, 1.0, 1.0);
```

### Error Estimation

Get estimation error for decision-making:

```c
float error = 0.0;
soc_calculator_get_estimation_error(module_id, &error);
// error now contains estimation error in percentage points
```

## Technical Notes

1. **Rest Detection**: For OCV-based methods, the battery should be at rest (current < 100mA) for accurate readings
2. **Initialization**: Without initial calibration, the system assumes a moderate initial SoC (50%)
3. **Efficiency Factors**: Charging efficiency of 97% is applied to account for energy losses
4. **Boundary Protection**: All SoC values are constrained to the 0-100% range regardless of calculation method
5. **Coulomb Counting Drift**: Long-term use requires periodic recalibration to combat integration drift

## Future Enhancements

- Machine learning-based SoC prediction models
- Adaptive filtering based on battery cycle history
- Enhanced aging models for more precise SoH estimation
- Self-learning OCV-SoC curve calibration
