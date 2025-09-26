# FTC Encoder Implementation Guide

## Overview
This guide explains the proper implementation of motor encoders in our autonomous framework, following FTC best practices and official documentation standards.

## FTC Encoder Best Practices Implementation

### 1. Proper Encoder Reset
```java
// CORRECT - FTC Best Practice
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// INCORRECT - Missing reset
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Doesn't reset to zero
```

### 2. Understanding Run Modes
- **STOP_AND_RESET_ENCODER**: Resets encoder count to zero and stops motor
- **RUN_WITHOUT_ENCODER**: Allows manual power control, encoders still work for reading
- **RUN_USING_ENCODER**: Uses built-in velocity control (PID)
- **RUN_TO_POSITION**: Moves motor to specific encoder position using PID

### 3. Encoder Value Calculations
```java
// Constants (replace with your robot's values)
double COUNTS_PER_MOTOR_REV = 537.7;        // GoBILDA 312 RPM motor
double DRIVE_GEAR_REDUCTION = 1.0;          // No external gearing
double WHEEL_DIAMETER_INCHES = 4.0;         // 4 inch wheels

// Calculated constants
double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                         (WHEEL_DIAMETER_INCHES * Math.PI);

// Reading values
int position = motor.getCurrentPosition();                    // Encoder counts
double revolutions = position / COUNTS_PER_MOTOR_REV;        // Revolutions
double angle = revolutions * 360.0;                         // Degrees (total)
double angleNormalized = angle % 360.0;                     // Degrees (0-360)
double distance = position / COUNTS_PER_INCH;               // Linear distance
```

## Updated Implementation in Framework

### AdvancedPositioningHelper Improvements

#### 1. Proper Encoder Initialization
```java
public void resetEncoders() {
    // Stop and reset all motor encoders
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // Turn motors back on for use (required after STOP_AND_RESET_ENCODER)
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
}
```

#### 2. Encoder Utility Methods
```java
// Get encoder values in different units
public int getEncoderCounts(String motor);           // Raw counts
public double getEncoderRevolutions(String motor);   // Revolutions
public double getEncoderInches(String motor);        // Linear distance
public double getEncoderAngle(String motor);         // Angle (0-360°)
public double getEncoderAngleTotal(String motor);    // Total angle
public String getEncoderTelemetry();                 // Formatted for display
public String getEncoderDistances();                 // Distances for display
```

#### 3. Improved Position Reset
```java
// Reset position and encoders
public void resetPosition(double x, double y, double heading);

// Set position without resetting encoders (for corrections)
public void setPosition(double x, double y, double heading);
```

### AutoHelper Improvements

#### 1. Encoder Reset at Initialization
```java
private void initializeMotors() {
    // ... motor setup ...
    
    // Reset encoder positions to zero (FTC best practice)
    resetEncoders();
    
    // Store initial positions (should be zero after reset)
    updateEncoderPositions();
}
```

#### 2. Same Utility Methods
AutoHelper now includes the same encoder utility methods as AdvancedPositioningHelper for consistency.

## Updated Field Constants

### FTC Official Field Dimensions
```java
// Updated to match official 144" x 144" field specification
public static final double FIELD_WIDTH = 144.0;         // 144" x 144" square field
public static final double FIELD_LENGTH = 144.0;        // 144" x 144" square field  
public static final double FIELD_CENTER_TO_WALL = 72.0; // 144/2 = 72 inches

// FTC Coordinate System positions
public static final double RED_ALLIANCE_X = -72.0;      // Red Wall X coordinate
public static final double BLUE_ALLIANCE_X = 72.0;      // Blue Wall X coordinate
public static final double AUDIENCE_Y = -72.0;          // Audience side Y coordinate
public static final double AWAY_FROM_AUDIENCE_Y = 72.0; // Away from audience Y coordinate
```

## Usage Examples

### 1. Basic Encoder Reading (TeleOp)
```java
@TeleOp(name = "Encoder Test")
public class EncoderTest extends OpMode {
    private DcMotor motor;
    
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "testMotor");
        
        // Reset encoder (FTC best practice)
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    @Override
    public void loop() {
        // Read encoder values
        int counts = motor.getCurrentPosition();
        double revolutions = counts / 537.7; // Replace with your CPR
        double angle = revolutions * 360.0;
        
        telemetry.addData("Encoder Counts", counts);
        telemetry.addData("Revolutions", String.format("%.2f", revolutions));
        telemetry.addData("Angle", String.format("%.1f°", angle));
        telemetry.update();
    }
}
```

### 2. RUN_TO_POSITION Usage (Autonomous)
```java
private void moveArmToPosition(int targetPosition, double power, double timeout) {
    // Set target position BEFORE setting mode
    armMotor.setTargetPosition(targetPosition);
    
    // Set to RUN_TO_POSITION mode
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    // Set power
    armMotor.setPower(Math.abs(power));
    
    // Wait for completion
    double startTime = getRuntime();
    while (opModeIsActive() && armMotor.isBusy() && 
           (getRuntime() - startTime) < timeout) {
        // Monitor progress
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", armMotor.getCurrentPosition());
        telemetry.addData("Error", targetPosition - armMotor.getCurrentPosition());
        telemetry.update();
        idle();
    }
    
    // Stop and return to manual mode
    armMotor.setPower(0);
    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
}
```

### 3. Using Framework Utility Methods
```java
// In your OpMode
AutoHelper autoHelper = new AutoHelper(this, telemetry);

// Get encoder values
int leftFrontCounts = autoHelper.getEncoderCounts("leftfront");
double leftFrontInches = autoHelper.getEncoderInches("leftfront");
double leftFrontRevs = autoHelper.getEncoderRevolutions("leftfront");

// Display all encoder data
telemetry.addData("Encoders", autoHelper.getEncoderTelemetry());
telemetry.addData("Distances", autoHelper.getEncoderDistances());
```

## Demo OpModes

### FTCEncoderDemo.java
- Demonstrates basic encoder reading
- Shows calculations for counts, revolutions, angles, distances
- Interactive with gamepad control
- Displays all encoder information in real-time

### FTCEncoderRunToPositionDemo.java
- Demonstrates RUN_TO_POSITION mode
- Shows arm and drive motor positioning
- Includes timeout and error handling
- Displays movement progress and accuracy

## Common Motor CPR Values

### Popular FTC Motors
```java
// GoBILDA Motors
double GOBILDA_312_RPM = 537.7;      // 312 RPM Yellow Jacket
double GOBILDA_223_RPM = 753.2;      // 223 RPM Yellow Jacket
double GOBILDA_117_RPM = 1425.1;     // 117 RPM Yellow Jacket

// REV Motors  
double REV_HD_HEX_40_1 = 1120.0;     // REV HD Hex 40:1
double REV_HD_HEX_20_1 = 560.0;      // REV HD Hex 20:1

// Tetrix Motors
double TETRIX_MOTOR = 1440.0;        // Tetrix TorqueNADO motor
```

### Calculating CPR for Geared Motors
```java
// Formula: Base Motor CPR × Gear Ratio
// Example: REV HD Hex motor with UltraPlanetary gearbox
double BASE_CPR = 28.0;              // REV HD Hex base motor
double GEAR_RATIO = 5.23;            // 5:1 UltraPlanetary (actual ratio)
double TOTAL_CPR = BASE_CPR * GEAR_RATIO; // 146.44
```

## Troubleshooting

### Common Issues
1. **Encoders not at zero**: Not calling `STOP_AND_RESET_ENCODER`
2. **Motor not moving after reset**: Not setting `RUN_WITHOUT_ENCODER` after reset
3. **Inaccurate distances**: Wrong CPR or wheel diameter values
4. **RUN_TO_POSITION not working**: Setting mode before target position

### Debug Tools
```java
// Check encoder status
telemetry.addData("Motor Mode", motor.getMode().toString());
telemetry.addData("Motor Busy", motor.isBusy());
telemetry.addData("Target Position", motor.getTargetPosition());
telemetry.addData("Current Position", motor.getCurrentPosition());
telemetry.addData("Power", motor.getPower());
```

This encoder implementation ensures reliable, accurate position tracking following FTC competition standards and best practices.