# Metrics Analyzer System

## Overview

The Metrics Analyzer is a critical component of the 100KW/200KWH Battery Energy Storage System (BESS) firmware. It provides comprehensive monitoring, analysis, and reporting capabilities for all system operational metrics. This component enables efficient performance tracking, trend analysis, and early problem detection to ensure optimal operation of the LFP battery modules.

## Core Functionality

### Metrics Collection

- **Continuous Data Sampling**: Captures operational metrics at configurable intervals
- **Multiple Metric Types**: Tracks up to 12 different metric categories including SoC, SoH, voltage, current, temperature
- **Time-Series Storage**: Maintains historical data for each metric type in efficient circular buffers
- **Multi-Level Collection**: Stores data at different time resolutions (real-time, minute, hour, day, week, month)

### Statistical Analysis

- **Comprehensive Statistics**: Calculates min, max, average, median, and standard deviation for each metric
- **Trend Detection**: Identifies whether metrics are stable, increasing, decreasing, or fluctuating
- **Rate of Change**: Determines how quickly metrics are changing over time
- **Correlation Analysis**: Examines relationships between different metrics (e.g., temperature vs. efficiency)

### Threshold Monitoring

- **Multi-Level Thresholds**: Configurable warning, critical, and emergency thresholds for each metric
- **Event Notification**: Triggers system-wide alerts when thresholds are crossed
- **Callback System**: Allows other system components to register for notifications
- **Alarm State Checking**: Enables quick determination of current system status

### Data Export & Reporting

- **SD Card Storage**: Archives metric data in JSON format for historical analysis
- **Cloud Integration**: Uploads metrics to AWS CloudWatch for remote monitoring
- **Performance Reports**: Generates comprehensive system performance reports
- **Data Visualization Support**: Provides structured data for dashboard displays

## Integration with BESS Components

The Metrics Analyzer integrates with other key BESS components:

### Battery Management System (BMS)

- Receives state of charge, state of health, and voltage data from the BMS
- Notifies BMS of unusual metric patterns that may indicate battery issues
- Provides historical data for BMS decisions on battery balancing and charge/discharge rates

### Thermal Monitor

- Tracks temperature metrics across the system
- Detects temperature trends that may indicate cooling system problems
- Provides historical temperature data to optimize thermal management

### Power Conversion System

- Monitors power flow, efficiency, and grid parameters
- Identifies conversion efficiency issues
- Tracks energy input/output for system performance evaluation

### System Controller

- Provides comprehensive operational overview
- Feeds data into system optimization algorithms
- Supports decision-making for energy dispatch

## Implementation Details

### Task Structure

The Metrics Analyzer uses a dedicated FreeRTOS task for continuous operation:

- Runs at a configurable interval (typically 500ms)
- Handles periodic data collection and analysis
- Manages automatic uploads to cloud storage
- Coordinates SD card data storage

### Data Synchronization

The implementation uses multiple synchronization mechanisms:

- **Mutex Protection**: Each metric type has a dedicated mutex for thread-safe access
- **Global Mutex**: Protects system-wide configuration changes
- **Event Groups**: Coordinates task operations and signals events

### Memory Management

- **Circular Buffers**: Efficient storage of time-series data without frequent allocations
- **Configurable Capacity**: Adjustable storage size per metric type
- **Automatic Pruning**: Oldest data is automatically overwritten when buffer is full

### Error Handling

- **Comprehensive Error Checking**: All functions return appropriate error codes
- **Timeout Protection**: All mutex operations have timeouts to prevent deadlocks
- **Graceful Degradation**: System continues operation even if some metrics are unavailable

## API Reference

The Metrics Analyzer exposes a comprehensive API for other system components:

### Initialization & Control

- `metrics_analyzer_init()`: Initialize the analyzer with configuration parameters
- `metrics_analyzer_deinit()`: Deinitialize and free resources
- `metrics_analyzer_start()`: Start the analyzer task
- `metrics_analyzer_stop()`: Stop the analyzer task

### Data Management

- `metrics_analyzer_add_data_point()`: Add a single metric data point
- `metrics_analyzer_add_data_points()`: Add multiple metric data points efficiently
- `metrics_analyzer_get_latest()`: Get the most recent value for a metric
- `metrics_analyzer_clear_history()`: Clear stored metric history

### Statistical Analysis

- `metrics_analyzer_get_statistics()`: Get comprehensive statistics for a metric
- `metrics_analyzer_get_single_statistic()`: Get a specific statistic (min, max, avg, etc.)
- `metrics_analyzer_get_trend()`: Determine the current trend for a metric

### Threshold Management

- `metrics_analyzer_set_thresholds()`: Configure threshold settings for a metric
- `metrics_analyzer_get_thresholds()`: Get current threshold settings
- `metrics_analyzer_register_threshold_callback()`: Register for threshold crossing events
- `metrics_analyzer_unregister_threshold_callback()`: Unregister from threshold events
- `metrics_analyzer_check_alarm_state()`: Check if a metric is in alarm state

### Reporting & Export

- `metrics_analyzer_export_to_json()`: Export metrics to a JSON file on the SD card
- `metrics_analyzer_force_cloud_upload()`: Force an immediate upload to AWS CloudWatch
- `metrics_analyzer_generate_performance_report()`: Generate a system performance report

## Configuration Options

Key configuration parameters for the Metrics Analyzer include:

- **Sampling Interval**: How frequently metrics are collected (ms)
- **Storage Capacity**: How many historical data points to store per metric
- **Cloud Upload**: Whether to enable AWS CloudWatch integration
- **Upload Interval**: How frequently to upload data to the cloud (seconds)
- **Log Path**: Where to store metrics logs on the SD card
- **Task Priority**: Priority of the analyzer task in the RTOS
- **Core Assignment**: Which CPU core to run the analyzer task on

## Example Usage

### Initialization

```c
// Create configuration
metrics_analyzer_config_t config = {
    .sampling_interval_ms = 500,
    .storage_capacity_per_metric = 10000,  // Store 10K points per metric
    .enable_cloud_upload = true,
    .cloud_upload_interval_s = 300,        // 5 minutes
    .metrics_log_path = "/sdcard/metrics",
    .task_priority = 10,
    .core_id = 0
};

// Initialize the analyzer
esp_err_t result = metrics_analyzer_init(&config);
if (result == ESP_OK) {
    // Start the analyzer
    metrics_analyzer_start();
}
```

### Adding Metrics

```c
// Add individual data points
metrics_analyzer_add_data_point(METRIC_TYPE_VOLTAGE, 48.2);
metrics_analyzer_add_data_point(METRIC_TYPE_CURRENT, 12.5);
metrics_analyzer_add_data_point(METRIC_TYPE_TEMPERATURE, 32.1);

// Add multiple data points at once (more efficient)
metrics_type_t types[3] = {
    METRIC_TYPE_SOC, METRIC_TYPE_POWER, METRIC_TYPE_EFFICIENCY
};
float values[3] = {
    78.5, 42.8, 94.2
};
metrics_analyzer_add_data_points(types, values, 3);
```

### Getting Statistics

```c
// Get comprehensive statistics
metrics_statistics_t stats;
metrics_analyzer_get_statistics(METRIC_TYPE_SOC, METRICS_PERIOD_DAY, &stats);

printf("SOC Statistics (24h):\n");
printf("  Min: %.1f%%\n", stats.min);
printf("  Max: %.1f%%\n", stats.max);
printf("  Avg: %.1f%%\n", stats.avg);
printf("  StdDev: %.2f\n", stats.stddev);
```

### Threshold Monitoring

```c
// Configure SOC thresholds
metrics_threshold_t soc_threshold = {
    .metric_type = METRIC_TYPE_SOC,
    .warning_low = 20.0f,
    .warning_high = 90.0f,
    .critical_low = 10.0f,
    .critical_high = 95.0f,
    .emergency_low = 5.0f,
    .emergency_high = 98.0f
};
metrics_analyzer_set_thresholds(&soc_threshold);

// Register threshold callback
void threshold_callback(metrics_type_t type, threshold_severity_t severity, 
                       float value, float threshold, void *user_data) {
    printf("Threshold crossed for metric %d: %.2f (threshold: %.2f, severity: %d)\n",
           type, value, threshold, severity);
           
    if (type == METRIC_TYPE_SOC && severity >= THRESHOLD_CRITICAL) {
        // Take emergency action for critical SOC condition
    }
}

metrics_analyzer_register_threshold_callback(
    METRIC_TYPE_SOC,
    THRESHOLD_WARNING,
    threshold_callback,
    NULL
);
```

### Generating Reports

```c
// Generate performance report
char report_buffer[4096];
size_t written_size;

metrics_analyzer_generate_performance_report(
    report_buffer,
    sizeof(report_buffer),
    &written_size
);

// Display or save the report
printf("%s\n", report_buffer);
```

### Exporting Data

```c
// Export all metrics to SD card
metrics_analyzer_export_to_json(
    METRIC_TYPE_MAX,  // All metrics
    METRICS_PERIOD_DAY,
    "system_metrics_daily.json"
);

// Force upload to CloudWatch
metrics_analyzer_force_cloud_upload(METRIC_TYPE_MAX);
```

## Future Enhancements

Planned improvements for the Metrics Analyzer include:

- **Machine Learning Integration**: Add predictive analytics for system behavior
- **Adaptive Thresholds**: Automatically adjust thresholds based on operational patterns
- **Advanced Anomaly Detection**: Identify unusual metric patterns before they trigger thresholds
- **Metric Correlation Analysis**: Advanced analysis of relationships between different metrics
- **Extended Data Visualization**: Direct generation of graphical reports
- **Multi-System Aggregation**: Compare metrics across multiple BESS deployments
