# Dead Wheel Odometry System for FTC

This system provides high-accuracy robot positioning using dedicated encoder wheels that don't slip, following the mathematics from Game Manual 0.

## Overview

Dead wheel odometry uses three encoder wheels:
- **Two parallel wheels** (left and right) for forward/backward and rotation tracking
- **One perpendicular wheel** (horizontal) for side-to-side tracking

This provides much higher accuracy than motor encoder odometry because:
- No wheel slip errors
- Higher resolution encoders
- Dedicated positioning sensors
- Better curved path tracking with pose exponentials

## Hardware Setup

### Required Components
- 3x Encoder wheels (omni wheels or similar)
- 3x Encoders (REV Through Bore, etc.)
- Mounting hardware for precise positioning

### Mounting Requirements
1. **Left/Right wheels**: Mount parallel to robot's forward direction
   - Should be as far apart as possible for better accuracy
   - Both should be same distance from center line
   
2. **Horizontal wheel**: Mount perpendicular to robot's forward direction
   - Position to minimize scrub during turns
   - Distance from center affects accuracy

### Hardware Map Configuration
```java
// In your hardware map configuration:
leftOdometer = hardwareMap.get(DcMotor.class, "leftOdometer");
rightOdometer = hardwareMap.get(DcMotor.class, "rightOdometer"); 
horizontalOdometer = hardwareMap.get(DcMotor.class, "horizontalOdometer");
```

## Software Integration

### Basic Usage

```java
// Initialize with default parameters
AdvancedPositioningHelper positioning = new AdvancedPositioningHelper(this);
positioning.initialize("Webcam 1", true); // true = use dead wheels

// Update position in your loop
while (opModeIsActive()) {
    positioning.updatePosition();
    
    // Use positioning data
    double x = positioning.getCurrentX();
    double y = positioning.getCurrentY();
    double heading = positioning.getCurrentHeading();
    
    positioning.updateTelemetry();
}
```

### Custom Parameters

```java
// Initialize with measured values for your robot
double trackWidth = 15.0;      // Distance between left/right wheels (inches)
double horizontalOffset = 6.0; // Distance from center to horizontal wheel (inches)

positioning.initialize("Webcam 1", true, trackWidth, horizontalOffset);
```

### Direct Dead Wheel Usage

```java
// Use DeadWheelOdometry directly for advanced control
DeadWheelOdometry odometry = new DeadWheelOdometry(hardwareMap, telemetry);

// Update position
odometry.updatePosition();

// Get position data
double[] position = odometry.getPosition(); // [x, y, heading_rad]
double[] velocity = odometry.getVelocity(); // [vx, vy, omega_rad/s]
```

## Calibration Process

### 1. Track Width Calibration

Use the `DeadWheelTuning` OpMode:

1. Place robot at center of field
2. Select "Track Width Tuning" mode
3. Press A to start test
4. Manually rotate robot exactly 10 full turns
5. Record the calculated track width value

**Alternative Manual Method:**
1. Measure physical distance between left and right dead wheel contact points
2. Add ~0.1-0.2 inches to account for encoder mounting and wheel compression

### 2. Horizontal Offset Calibration

Using `DeadWheelTuning` OpMode:

1. Place robot at known position  
2. Select "Horizontal Offset Tuning" mode
3. Press A to start test
4. Manually strafe robot exactly 48 inches sideways
5. Keep heading constant during strafe
6. Record the calculated horizontal offset value

**Alternative Manual Method:**
1. Measure distance from robot's center of rotation to horizontal wheel contact point
2. Positive if wheel is forward of center, negative if behind

### 3. Encoder Direction Verification

1. Select "Encoder Direction Test" mode
2. Press A to start 10-second test
3. Push robot forward slowly
4. Verify:
   - Left encoder increases (positive)
   - Right encoder increases (positive) 
   - Horizontal encoder stays near zero

If directions are wrong, either:
- Reverse encoder direction in hardware map: `encoder.setDirection(DcMotorSimple.Direction.REVERSE)`
- Or modify the direction multipliers in `DeadWheelOdometry.java`

## Tuning Tips

### Accuracy Troubleshooting

**Position drifts during straight movement:**
- Check encoder directions
- Verify wheels are properly aligned
- Ensure wheels aren't slipping on mounting

**Rotation accuracy issues:**
- Recalibrate track width
- Check that left/right wheels are equidistant from center
- Verify encoder resolution settings

**Strafing accuracy issues:**
- Recalibrate horizontal offset
- Check horizontal wheel alignment
- Ensure wheel isn't binding during movement

### Performance Optimization

**For maximum accuracy:**
- Use highest resolution encoders available
- Mount wheels as rigidly as possible
- Keep wheels clean and free of debris
- Calibrate regularly (temperature affects dimensions)

**For better curved path tracking:**
- The system uses pose exponentials automatically
- No additional tuning needed for curved paths
- System handles all FTC movement patterns correctly

## Code Structure

### DeadWheelOdometry.java
- Core odometry mathematics
- Three-wheel odometry algorithms
- Pose exponential calculations
- Sensor fusion with IMU
- Real-time position and velocity tracking

### AdvancedPositioningHelper.java  
- High-level positioning interface
- Automatic fallback to motor encoders
- Integration with AprilTag vision
- Go-to-position and path following
- Sensor fusion between all positioning sources

### Demo OpModes
- `DeadWheelOdometryDemo.java`: Basic usage demonstration
- `DeadWheelTuning.java`: Calibration and tuning tools

## DECODE Field Coordinate System Compliance

The system uses the official FTC coordinate system for DECODE field configuration:
- **Origin**: Field center (0,0)
- **X-axis**: Back of field (-X) to Audience (+X), positive toward audience
- **Y-axis**: Red Wall/Red Alliance (-Y) to Blue Wall/Blue Alliance (+Y)
- **Heading**: 0° = facing positive Y (toward Blue Alliance), 90° = facing positive X (toward audience)
- **Rotations**: Positive = counter-clockwise
- **Alliance Areas**: Red Alliance on LEFT (negative Y), Blue Alliance on RIGHT (positive Y) when viewed from audience

**DECODE Field Specifics:**
- Field dimensions: 141" x 141" (3580mm inside perimeter)
- Red Wall on left side when viewed from audience (inverted configuration)
- Example coordinate: Red Goal AprilTag at (-58.3727, 55.6425, 29.5)

All position data is in inches and degrees for easy field navigation.

## Constants to Adjust

In your robot constants file:

```java
// Dead Wheel Odometry Constants (measure and calibrate these)
public static final double DEAD_WHEEL_TRACK_WIDTH = 15.0;        // inches
public static final double DEAD_WHEEL_HORIZONTAL_OFFSET = 6.0;   // inches

// Encoder specifications (check your encoder documentation)
public static final double DEAD_WHEEL_COUNTS_PER_REV = 2048.0;   // REV Through Bore
public static final double DEAD_WHEEL_DIAMETER = 2.0;            // inches (omni wheel)

// Calculated automatically
public static final double DEAD_WHEEL_COUNTS_PER_INCH = 
    DEAD_WHEEL_COUNTS_PER_REV / (DEAD_WHEEL_DIAMETER * Math.PI);
```

## Advanced Features

### Sensor Fusion
- Automatically combines dead wheels with IMU heading
- AprilTag corrections for absolute position
- Weighted sensor fusion based on confidence

### Automatic Fallback
- Falls back to motor encoders if dead wheels fail
- Seamless transition with no code changes needed
- Error reporting and status monitoring

### Real-time Diagnostics
- Encoder health monitoring
- Position accuracy tracking
- Velocity and acceleration data
- Debug telemetry for troubleshooting

## Troubleshooting

### Common Issues

**"Hardware not found" error:**
- Check hardware map names match your configuration
- Verify all three encoders are connected
- Check encoder ports and wiring

**Position jumps or erratic behavior:**
- Check for loose encoder connections
- Verify wheel mounting is rigid
- Look for electrical interference

**Gradual position drift:**
- Recalibrate track width and horizontal offset
- Check for wheel wear or damage
- Verify encoder directions are correct

**Poor accuracy:**
- Ensure wheels aren't slipping on mounting surface
- Check encoder resolution and diameter settings
- Recalibrate using tuning OpMode

### Getting Help

1. Run `DeadWheelTuning` OpMode for diagnostics
2. Check telemetry for error messages and status
3. Verify hardware connections and configuration
4. Review calibration values and re-measure if needed

The dead wheel odometry system provides the highest accuracy positioning available in FTC when properly calibrated and maintained.