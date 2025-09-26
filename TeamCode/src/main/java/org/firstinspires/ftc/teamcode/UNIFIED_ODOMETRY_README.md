# Unified Dead Wheel Odometry System

## Overview

This enhanced odometry system supports both traditional 3-wheel dead wheel systems and goBILDA's advanced 2-pod Pinpoint odometry system. The system automatically detects which hardware is available and configures accordingly, providing a unified interface for both systems.

## Supported Systems

### 1. Traditional 3-Wheel Dead Wheel System
- **Hardware**: Three dead wheel encoders (left, right, horizontal)
- **Features**: Proven reliability, manual tuning, works with any encoder
- **Best For**: Custom builds, budget-conscious teams, maximum control

### 2. goBILDA Pinpoint 2-Pod System  
- **Hardware**: Pinpoint Odometry Computer + two odometry pods
- **Features**: High accuracy, built-in IMU, automatic calibration, velocity data
- **Best For**: Competition teams, plug-and-play solution, advanced features

## Hardware Configuration

### Traditional System
```
Hardware Map Names:
- "leftOdometry": Left dead wheel encoder
- "rightOdometry": Right dead wheel encoder  
- "horizontalOdometry": Horizontal dead wheel encoder
```

### Pinpoint System
```
Hardware Map Names:
- "pinpoint": goBILDA Pinpoint Odometry Computer (I2C device)

Physical Setup:
- X Pod: Forward-tracking odometry pod
- Y Pod: Strafe-tracking odometry pod
- Both pods connected to Pinpoint computer
- Pinpoint connected to robot via I2C
```

## DECODE Field Configuration

The system is pre-configured for DECODE field coordinates:

- **Field Size**: 141" × 141" square
- **Origin**: Field center (0, 0)
- **X-Axis**: Back of field (-X) to Audience (+X)
- **Y-Axis**: Red Alliance (-52.5") to Blue Alliance (+52.5")
- **Heading**: 0° = facing toward Blue Alliance (positive Y)

### Key Field Positions:
- **Red Alliance Start**: (-48", -48", 0°)
- **Blue Alliance Start**: (-48", +48", 0°)
- **Field Center**: (0", 0", 0°)
- **Audience Side**: (+60", 0", 0°)

## Usage Examples

### Basic Initialization (Auto-Detection)
```java
// Automatically detects available hardware
DeadWheelOdometry odometry = new DeadWheelOdometry(hardwareMap, telemetry);

if (odometry.isInitialized()) {
    telemetry.addData("System Type", odometry.getSystemType());
    telemetry.addData("Status", odometry.getSystemStatus());
}
```

### Force Specific System
```java
// Force traditional 3-wheel system with custom parameters
DeadWheelOdometry odometry = new DeadWheelOdometry(
    hardwareMap, telemetry, 
    12.0,  // trackWidth in inches
    6.0    // horizontalOffset in inches
);

// Force Pinpoint system with custom pod offsets
DeadWheelOdometry odometry = new DeadWheelOdometry(
    hardwareMap, telemetry,
    -3.31,  // X pod offset (inches, left positive)
    -6.61,  // Y pod offset (inches, forward positive)
    true    // use Pinpoint system
);
```

### Position Tracking
```java
while (opModeIsActive()) {
    // Update position (call every loop)
    odometry.updatePosition();
    
    // Get current position
    double x = odometry.getX();              // inches
    double y = odometry.getY();              // inches  
    double heading = odometry.getHeadingDegrees(); // degrees
    
    // Get velocity (Pinpoint only)
    if (odometry.isPinpointSystem()) {
        double[] velocity = odometry.getVelocity();
        double vx = velocity[0];    // inches/sec
        double vy = velocity[1];    // inches/sec
        double omega = velocity[2]; // radians/sec
    }
}
```

### Position Control
```java
// Reset to origin
odometry.resetPosition();

// Reset to specific position
odometry.resetPosition(-48.0, -48.0, 0.0); // Red alliance start

// Set position without encoder reset (for corrections)
odometry.setPosition(0.0, 0.0, 90.0);
```

### Navigation Helpers
```java
// Calculate distance and angle to target
double distance = odometry.getDistanceToPoint(targetX, targetY);
double angle = odometry.getAngleToPoint(targetX, targetY);

// Static distance calculation
double dist = DeadWheelOdometry.calculateDistance(x1, y1, x2, y2);
```

## System-Specific Features

### Traditional 3-Wheel System
```java
// Tuning parameters (must be measured for your robot)
odometry.setTrackWidth(12.5);        // Distance between left/right wheels
odometry.setHorizontalOffset(6.25);   // Forward offset of horizontal wheel

// Get current parameters
double trackWidth = odometry.getTrackWidth();
double offset = odometry.getHorizontalOffset();
```

### Pinpoint System Only
```java
// Check system type
if (odometry.isPinpointSystem()) {
    // Recalibrate IMU
    odometry.recalibrateIMU();
    
    // Get Pinpoint-specific data
    double frequency = pinpointAdapter.getFrequency();  // Update frequency
    int loopTime = pinpointAdapter.getLoopTime();       // Loop time in μs
    String status = pinpointAdapter.getStatus();        // Detailed status
    
    // Configure encoder resolution
    odometry.setPinpointEncoderResolution(19.894); // ticks per mm
}
```

## Telemetry and Debugging

### Basic Telemetry
```java
// Add comprehensive odometry telemetry
odometry.addTelemetry();

// Manual telemetry
telemetry.addData("Position", odometry.getFormattedPosition());
telemetry.addData("System Type", odometry.getSystemType());
telemetry.addData("Update Rate", odometry.getUpdateFrequency() + " Hz");
```

### Diagnostic Information
```java
// Get full diagnostic report
String diagnostics = odometry.getDiagnostics();
telemetry.addData("Diagnostics", diagnostics);

// Check system status
telemetry.addData("Status", odometry.getSystemStatus());
telemetry.addData("Initialized", odometry.isInitialized());
```

## Tuning Guide

### Traditional 3-Wheel System
1. **Measure Track Width**: Distance between left and right dead wheels
2. **Measure Horizontal Offset**: Forward distance of horizontal wheel from robot center
3. **Test Forward Movement**: Ensure X increases when robot moves forward
4. **Test Lateral Movement**: Ensure Y increases when robot moves left
5. **Test Rotation**: Robot should return to same position after full rotation

### Pinpoint System
1. **Mount Pinpoint**: Sticker-side up on robot chassis
2. **Connect Pods**: X pod tracks forward, Y pod tracks sideways
3. **Configure Offsets**: Distance from pods to tracking point
4. **Test Directions**: X encoder increases forward, Y encoder increases left
5. **Verify IMU**: Heading should be stable when robot is stationary

## Troubleshooting

### Common Issues

**No System Detected**
- Check hardware connections
- Verify hardware map names match configuration
- Ensure proper I2C connection for Pinpoint

**Poor Accuracy**
- Traditional: Verify track width and horizontal offset measurements
- Pinpoint: Check pod offsets and encoder directions
- Both: Ensure dead wheels aren't slipping

**Drift Issues**
- Traditional: Check for loose encoder connections
- Pinpoint: Recalibrate IMU when robot is stationary
- Both: Verify robot is level and stable

**Update Rate Issues**
- Target: >50 Hz for good performance
- If low: Check for blocking operations in main loop
- Pinpoint should achieve 1500 Hz internal updates

### Diagnostic Commands
```java
// System detection
telemetry.addData("System Type", odometry.getSystemType());
telemetry.addData("Initialized", odometry.isInitialized());

// Performance monitoring  
telemetry.addData("Update Rate", odometry.getUpdateFrequency());

// Encoder data
telemetry.addData("Raw Encoders", odometry.getEncoderPositions());
telemetry.addData("Encoder Inches", odometry.getEncoderInches());

// Full diagnostics
telemetry.addData("Diagnostics", odometry.getDiagnostics());
```

## Integration with Existing Code

The unified system maintains backward compatibility with existing `DeadWheelOdometry` implementations while adding new features:

- All existing method calls continue to work
- New methods are available for system-specific features
- Automatic hardware detection eliminates manual configuration
- Same coordinate system and conventions

## Files Included

- `DeadWheelOdometry.java`: Main unified odometry class
- `PinpointOdometryAdapter.java`: Adapter for Pinpoint system
- `UnifiedOdometryDemo.java`: Demonstration OpMode
- `UNIFIED_ODOMETRY_README.md`: This documentation

## Performance Comparison

| Feature | Traditional 3-Wheel | Pinpoint 2-Pod |
|---------|-------------------|----------------|
| Accuracy | ±0.5" typical | ±0.25" typical |
| Update Rate | 50-100 Hz | 1500 Hz internal |
| Setup Complexity | Medium | Low |
| Cost | Low | Medium |
| Velocity Data | No | Yes |
| IMU Integration | External | Built-in |
| Calibration | Manual | Automatic |

## Conclusion

This unified odometry system provides the best of both worlds - the proven reliability of traditional dead wheel systems and the advanced features of modern integrated solutions. Teams can choose the system that best fits their needs, budget, and technical requirements while using the same software interface.