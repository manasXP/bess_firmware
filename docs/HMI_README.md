# BESS HMI (Human-Machine Interface)

## Overview

This document outlines the design specifications for the Human-Machine Interface (HMI) of the 100KW/200KWH Battery Energy Storage System (BESS). The interface is designed for a 10-inch touchscreen display connected to the main ESP32-P4 controller, providing comprehensive monitoring and control capabilities for the BESS system with LFP battery modules (48V, 16KWH).

## Development Libraries

For implementing the HMI on the ESP32-P4 platform, the following libraries are recommended:

### Primary UI Framework Options

1. **LVGL (Light and Versatile Graphics Library)**
   - **Recommendation**: Strongly recommended for ESP32-P4 development
   - **Advantages**:
     - Designed specifically for embedded systems
     - Memory-efficient with low CPU usage
     - Hardware acceleration support for ESP32-P4's GPU
     - Comprehensive widget library (buttons, charts, gauges, etc.)
     - Touch input handling
     - Free and open-source (MIT license)
   - **Integration**: 
     - Compatible with ESP-IDF
     - Built-in ESP32 drivers for common displays
     - Supports development on FreeRTOS

2. **Squareline Studio**
   - **Description**: A visual GUI designer specifically for LVGL
   - **Advantages**:
     - Drag-and-drop interface
     - Exports LVGL C code
     - Speeds up development process significantly
     - Themes and built-in design elements

3. **ESP-IDF Component: esp_lcd**
   - **Description**: Direct ESP-IDF component for LCD control
   - **Use case**: Can be used alongside LVGL for direct hardware control

### Supporting Libraries

1. **ESP-DSP**
   - For efficient data processing and calculations needed for graphs and analytics
   
2. **esp-idf-lvgl-drivers**
   - Collection of display and touch drivers optimized for ESP-IDF and LVGL

3. **FT6X36-ESP** or similar touch controller libraries
   - For touch input handling based on the specific touch controller

4. **MicroPython UI options** (if using MicroPython instead of native C/C++)
   - lv_micropython: LVGL bindings for MicroPython
   - ulab: NumPy-like fast vector operations

## General Design Principles

- **Modern, Clean Interface**: Minimalist design with high contrast for readability
- **Responsive Layout**: Optimized for 10-inch touchscreen with appropriate touch targets
- **Color Scheme**: 
  - Primary: Deep blue (#1a5276) for headers and navigation
  - Secondary: White/light gray background for readability
  - Accent: Green (#2ecc71) for normal status, Yellow (#f39c12) for warnings, Red (#c0392b) for alarms
- **Consistent Navigation**: Fixed navigation bar at bottom of screen
- **Status Indicators**: Persistent system status indicators at top of screen

## Screen Designs

### 1. Dashboard (Home Screen)

**Purpose**: Provide an at-a-glance overview of the entire BESS system status

**Elements**:
- **Header Bar**:
  - System name/ID
  - Current date and time
  - Connection status indicators (Cloud, Modbus, CANBus)
  - Alarm indicator (with count)

- **Main Status Panel**:
  - Large circular gauge showing system SoC (0-100%)
  - Current operating mode indicator (Standby, Charging, Discharging, etc.)
  - Current power flow indicator with directional arrow (kW)
  
- **Key Metrics Tiles**:
  - System voltage (V)
  - System current (A)
  - Maximum cell temperature (°C)
  - System state of health (%)
  
- **Quick Action Buttons**:
  - Start/Stop operation
  - Switch mode (Charge/Discharge)
  - Emergency stop (protected)
  
- **Power Flow Diagram**:
  - Animated visual showing power flow between grid, battery, and load
  - Real-time power values on each flow path
  - Color-coded status for each component

- **Recent Alarms/Events**:
  - Scrollable list of most recent system events
  - Color-coded by severity
  - Timestamp and brief description

### 2. Battery Status Screen

**Purpose**: Detailed view of all battery modules and cells

**Elements**:
- **Module Overview Grid**:
  - Visual representation of all modules (4×3 grid for 12 modules)
  - Color-coded by status/temperature
  - Each module shows voltage, temperature, and SoC
  
- **Selected Module Details**:
  - Expanded view when a module is tapped
  - Individual cell voltages displayed as bar chart
  - Cell temperature map with color gradient
  - Cell balance status indicators
  
- **Module Selector**:
  - Quick-access carousel to select modules
  - Color-coded by health/alarm status
  
- **Historical Data Graph**:
  - Plotted history of module parameters
  - Selectable time ranges (1h, 24h, 7d)
  - Multiple parameters (voltage, temperature, SoC)

- **Balance Control Panel**:
  - Balance status indicator
  - Start/Stop balance button
  - Balance mode selector (passive/active/hybrid)

### 3. Thermal Management Screen

**Purpose**: Monitor and control thermal conditions across all modules

**Elements**:
- **Temperature Overview**:
  - Heat map visualization of all modules
  - Gradient color scale showing temperature distribution
  
- **Temperature Metrics**:
  - Maximum temperature (with location)
  - Minimum temperature (with location)
  - Average temperature
  - Ambient temperature
  - Delta temperature (max - ambient)
  
- **Thermal Zone Indicator**:
  - Current thermal zone (Normal, Elevated, Warning, Critical, Emergency)
  - Color-coded zone indicator
  
- **Cooling System Control**:
  - Current cooling method indicator (Passive, Forced Air, Liquid)
  - Cooling power percentage with slider control
  - Auto/Manual mode switch
  - Fan speed control (if applicable)
  
- **Temperature History**:
  - Graph of temperature trends over time
  - Multiple series for max, min, average
  - Time range selector

- **Thermal Runaway Protection**:
  - Runaway detection status
  - Temperature rate-of-change indicator
  - Threshold settings display

### 4. Power Management Screen

**Purpose**: Control and monitor power flow settings

**Elements**:
- **Operating Mode Control**:
  - Mode selection buttons (Standby, Charge, Discharge, etc.)
  - Current mode indicator
  
- **Power Setpoints**:
  - Charge power limit (slider and numeric input)
  - Discharge power limit (slider and numeric input)
  - Current power usage indicator (gauge)
  
- **Grid Connection**:
  - Connection status indicator
  - Connect/Disconnect grid button
  - Island mode toggle switch
  
- **Schedule Management**:
  - Daily schedule view (timeline)
  - Quick schedule entry creator
  - Schedule enable/disable toggle
  
- **Power Statistics**:
  - Today's energy charged (kWh)
  - Today's energy discharged (kWh)
  - Total energy throughput (kWh)
  - Efficiency metrics (%)

- **Power Graphs**:
  - Power flow over time
  - Grid import/export patterns
  - Load consumption patterns

### 5. Alarm & Event Screen

**Purpose**: Review and manage system alarms and events

**Elements**:
- **Active Alarms Panel**:
  - Scrollable list of current active alarms
  - Sorted by severity and time
  - Color-coded by level (Warning, Error, Critical)
  - Clear/Acknowledge buttons for each alarm
  
- **Alarm History**:
  - Searchable log of historical alarms/events
  - Filter options (severity, module, time range)
  - Export function for logs
  
- **Alarm Statistics**:
  - Count by type pie chart
  - Frequency analysis
  - Most common alarms
  
- **Safety Status Panel**:
  - Emergency stop status
  - Protection systems status
  - Failsafe indicators
  
- **Notification Settings**:
  - Alert sound enable/disable
  - Remote notification options (if applicable)
  - Alarm threshold adjustments

### 6. System Configuration Screen

**Purpose**: Adjust system parameters and access advanced settings

**Elements**:
- **Battery Settings Panel**:
  - SoC limits (min/max)
  - Voltage thresholds
  - Current limits
  - Temperature thresholds
  
- **Communication Settings**:
  - Modbus configuration
  - CANBus configuration
  - Cloud connectivity settings
  - Network parameters
  
- **Logging Configuration**:
  - Log level selector
  - Log destination checkboxes
  - Log rotation settings
  
- **User Access**:
  - User management
  - Access level controls
  - Password change
  
- **System Information**:
  - Firmware version
  - Hardware details
  - System uptime
  - Last maintenance date

- **Advanced Settings** (password protected):
  - Calibration controls
  - Factory reset option
  - Debug mode toggle

### 7. Diagnostics Screen

**Purpose**: System testing and troubleshooting

**Elements**:
- **Self-Test Panel**:
  - Run diagnostics button
  - Component-specific test buttons
  - Test results display
  
- **System Health**:
  - CPU usage meter
  - Memory usage meter
  - Storage usage meter
  - Task manager view
  
- **Sensor Readings**:
  - Raw sensor values table
  - Sensor status indicators
  - Calibration status
  
- **Communication Test**:
  - Ping test results
  - Modbus status
  - CANBus status
  - Cloud connectivity test
  
- **Logs Viewer**:
  - Console log display
  - Log level filter
  - Search functionality
  - Download logs button

### 8. Firmware Update Screen

**Purpose**: Manage system firmware updates

**Elements**:
- **Current Version Panel**:
  - Installed firmware version
  - Release date
  - Update status
  
- **Available Updates Panel**:
  - New version details (if available)
  - Release notes
  - Download size and estimated time
  
- **Update Controls**:
  - Check for updates button
  - Download update button
  - Install update button
  - Progress indicator
  
- **Update History**:
  - List of previously installed updates
  - Installation dates
  - Success/failure status
  
- **Update Settings**:
  - Automatic check toggle
  - Update source selection
  - Schedule update option

### 9. Energy Management Screen

**Purpose**: Monitor and analyze energy patterns and efficiency

**Elements**:
- **Energy Flow Diagram**:
  - Sankey diagram showing energy flows
  - Real-time and cumulative energy metrics
  
- **Energy Statistics**:
  - Daily/weekly/monthly energy totals
  - Charge/discharge cycles count
  - Efficiency calculations
  
- **Energy Graphs**:
  - Stacked bar chart of daily energy patterns
  - Charge/discharge cycle visualization
  - Trend analysis over time
  
- **Energy Calendar**:
  - Monthly view of energy patterns
  - Peak day highlighting
  - Energy balance by day
  
- **Efficiency Metrics**:
  - Round-trip efficiency calculation
  - Loss analysis
  - Performance compared to baseline

## Navigation Design

**Bottom Navigation Bar**:
- Home/Dashboard (always visible)
- Battery Status
- Thermal Management
- Power Management
- More (dropdown for additional screens)
- Settings

**Status Bar (Top)**:
- System status indicator (color-coded)
- Active mode display
- Current power flow (kW with direction)
- SoC percentage
- Critical alarm indicator
- Time and date

## Responsive Elements

- **Orientation Support**: Primarily designed for landscape mode, with limited portrait support
- **Touch Gestures**: Swipe between related screens, pinch to zoom graphs
- **Contextual Controls**: Options that appear based on system state
- **Error Prevention**: Confirmation dialogs for critical actions

## Additional UI Features

- **Night Mode**: Reduced brightness and dark theme for overnight monitoring
- **User Levels**: Different access levels (Viewer, Operator, Administrator) with permission control
- **Context-Sensitive Help**: Help icons that provide explanations for complex metrics
- **Responsive Alarms**: Visual and audible notifications for critical events
- **Interactive Elements**: Tooltips on hover, expandable panels for detailed information

## Implementation Recommendations

### Hardware Considerations

1. **Display Selection**:
   - IPS panel for wide viewing angles
   - 1280×800 resolution minimum
   - Capacitive touch with multi-touch support
   - Sunlight readability for outdoor installations
   - Optional anti-glare coating

2. **Display Interface**:
   - RGB interface for higher performance
   - Or SPI/QSPI for simpler integration
   - Touch controller via I2C

3. **Memory Requirements**:
   - Minimum 8MB PSRAM for frame buffer and UI assets
   - SPIFFS/LittleFS for storing UI assets
   - Consider external flash for storing historical data

### Performance Optimization

1. **Graphics Acceleration**:
   - Utilize ESP32-P4's GPU capabilities where possible
   - Implement frame buffering for smoother transitions
   - Use hardware sprites for animated elements

2. **Memory Management**:
   - Implement partial screen updates to reduce memory usage
   - Use compressed images for UI assets
   - Separate RAM allocation for critical system functions

3. **Responsiveness**:
   - Prioritize UI task to ensure consistent frame rate
   - Use separate cores for UI and system functions
   - Implement background data processing for analytics

## Development Approach

1. **Phased Implementation**:
   - Phase 1: Core screens (Dashboard, Battery Status, Alarms)
   - Phase 2: Control screens (Power Management, Thermal Management)
   - Phase 3: Advanced features (Energy Management, Diagnostics)

2. **Testing Strategy**:
   - Unit tests for UI components
   - Simulated data for interface testing
   - User testing for ergonomics and usability

3. **Deployment**:
   - OTA update capability for UI components
   - Version control for UI assets
   - Customizable elements for different deployments

## Integration with BESS Firmware

1. **Data Exchange**:
   - Use shared memory approach for most efficient data transfer
   - Implement circular buffers for time-series data
   - Define clear API for firmware-UI interaction

2. **Event Handling**:
   - Register for system events to trigger UI updates
   - Implement data validation for user inputs
   - Define priority scheme for critical alerts

3. **Safety Considerations**:
   - Implement timeout for critical control actions
   - Require confirmation for potentially harmful operations
   - Design UI to prevent accidental activation of emergency functions
