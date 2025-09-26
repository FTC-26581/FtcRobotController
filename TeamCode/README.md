# Advanced Positioning Helper (APH) & AutoHelper Autonomous Framework (AAF)

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Quick Start Guide](#quick-start-guide)
3. [AutoHelper Framework (AAF) API](#autohelper-framework-aaf-api)
4. [Advanced Positioning Helper (APH) API](#advanced-positioning-helper-aph-api)
5. [Movement Traversal Modes](#movement-traversal-modes)
6. [Field Coordinate System](#field-coordinate-system)
7. [Configuration Examples](#configuration-examples)
8. [Troubleshooting](#troubleshooting)
9. [Sample OpModes](#sample-opmodes)

---

## 🚀 System Overview

The **Advanced Positioning Helper (APH)** and **AutoHelper Autonomous Framework (AAF)** provide a comprehensive, professional-grade autonomous system for FTC robots. This system supports multiple positioning technologies and provides intelligent error detection, recovery, and movement optimization.

### Core Features
- **🎯 Multi-System Positioning**: Dead wheel odometry, Pinpoint 2-pod, motor encoders, AprilTag vision
- **📷 Dual Camera Support**: Front and back cameras with intelligent switching
- **🧠 Smart Error Recovery**: Automatic stuck detection, AprilTag relocalization, timeout handling
- **🛣️ Movement Traversal Modes**: 5 different movement strategies for obstacle avoidance
- **⚡ Fluent API**: Chain commands for readable, maintainable autonomous code
- **📊 Comprehensive Telemetry**: Real-time debugging and performance monitoring

### Package Structure
```
org.firstinspires.ftc.teamcode/
├── util/                           # Framework classes
│   ├── AutoHelper.java            # Main autonomous framework
│   ├── AdvancedPositioningHelper.java  # Positioning system
│   ├── DeadWheelOdometry.java     # Dead wheel odometry
│   ├── PinpointOdometryAdapter.java    # Pinpoint system adapter
│   └── AprilTagMultiTool.java     # AprilTag utilities
├── samples/
│   ├── autonomous/                # Example OpModes
│   └── templates/                 # Copy-paste templates
```

---

## ⚡ Quick Start Guide

### 1. Basic Autonomous OpMode Template

```java
import org.firstinspires.ftc.teamcode.util.AutoHelper;

@Autonomous(name="My Autonomous", group="Team")
public class MyAutonomous extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        // Initialize framework
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1", "Webcam 2");  // Dual cameras
        
        // Configure features
        autoHelper.setRelocalizationEnabled(true);
        autoHelper.setStuckDetectionEnabled(true);
        autoHelper.setDefaultParameters(0.6, 5000);  // 60% power, 5s timeout
        
        waitForStart();
        
        // Autonomous routine using fluent API
        autoHelper
            .moveTo(24, 36, 90, "Move to scoring position")
            .waitFor(1000, "Wait for mechanism")
            .turnTo(45, "Face basket")
            .moveBy(-6, 0, 0, "Back away")
            .executeAll();
            
        telemetry.addData("Result", autoHelper.isExecutionComplete() ? "SUCCESS" : "FAILED");
        telemetry.update();
    }
}
```

### 2. Position Setup Options

```java
// Option 1: Motor encoders + dual cameras (recommended)
autoHelper.initialize("Webcam 1", "Webcam 2");

// Option 2: Motor encoders + single camera
autoHelper.initialize("Webcam 1");

// Option 3: Motor encoders only (no cameras)
autoHelper.initialize(null, null);
autoHelper.getPositioningHelper().resetPosition(-12, -52.5, 90);  // Set start position

// Option 4: Try dead wheels, fallback to encoders
autoHelper.initialize("Webcam 1", "Webcam 2", true);  // attemptDeadWheels = true
```

---

## 🤖 AutoHelper Framework (AAF) API

### Constructor & Initialization

#### `AutoHelper(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry)`
Creates new AutoHelper instance.

#### Initialization Methods
```java
// Basic initialization (motor encoders only)
void initialize()

// Single camera initialization  
void initialize(String webcamName)

// Dual camera initialization (recommended)
void initialize(String frontWebcamName, String backWebcamName)
```

### Configuration Methods (Fluent API)

#### `AutoHelper setRelocalizationEnabled(boolean enabled)`
Enables/disables AprilTag-based position correction during movement.
- **Default**: `true`
- **Recommended**: `true` for competition reliability

#### `AutoHelper setStuckDetectionEnabled(boolean enabled)`
Enables/disables automatic stuck detection and recovery.
- **Default**: `true`
- **Recommended**: `true` for obstacle handling

#### `AutoHelper setDebugTelemetryEnabled(boolean enabled)`
Enables/disables detailed telemetry output.
- **Default**: `false`
- **Recommended**: `true` for tuning, `false` for competition

#### `AutoHelper setDefaultParameters(double power, double timeoutMs)`
Sets default power and timeout for all movements.
- **power**: 0.0 to 1.0 (recommend 0.5-0.7)
- **timeoutMs**: Maximum time per movement (recommend 3000-7000ms)

#### `AutoHelper setStuckDetectionParameters(double thresholdInches, double timeThresholdMs)`
Configures stuck detection sensitivity.
- **thresholdInches**: Minimum movement to avoid "stuck" (default: 2.0)
- **timeThresholdMs**: Time threshold for stuck detection (default: 1500ms)

#### `AutoHelper setDefaultTraversalMode(MovementTraversalMode mode)`
Sets default movement traversal strategy (see [Movement Traversal Modes](#movement-traversal-modes)).

### Movement Commands (Fluent API)

#### Absolute Movement
```java
// Basic movement with default parameters
AutoHelper moveTo(double x, double y, double heading, String description)

// Movement with custom power and timeout
AutoHelper moveTo(double x, double y, double heading, double power, long timeoutMs, String description)

// Movement with specific traversal mode
AutoHelper moveTo(double x, double y, double heading, MovementTraversalMode traversalMode, String description)

// Full control movement
AutoHelper moveTo(double x, double y, double heading, double power, long timeoutMs, MovementTraversalMode traversalMode, String description)
```

#### Relative Movement
```java
// Basic relative movement
AutoHelper moveBy(double deltaX, double deltaY, double deltaHeading, String description)

// Relative movement with custom parameters
AutoHelper moveBy(double deltaX, double deltaY, double deltaHeading, double power, long timeoutMs, String description)
```

#### Rotation
```java
// Basic rotation to absolute heading
AutoHelper turnTo(double heading, String description)

// Rotation with custom parameters
AutoHelper turnTo(double heading, double power, long timeoutMs, String description)

// Relative rotation
AutoHelper turnBy(double deltaHeading, String description)
AutoHelper turnBy(double deltaHeading, double power, long timeoutMs, String description)
```

#### Timing & Custom Actions
```java
// Wait for specified time
AutoHelper waitFor(long milliseconds, String description)

// Execute custom action
AutoHelper customAction(String description, Supplier<Boolean> action)

// Conditional execution
AutoHelper executeIf(String description, Supplier<Boolean> condition, Supplier<Boolean> action)
```

### Execution Control

#### `void executeAll()`
Executes all queued commands in sequence. Handles errors, timeouts, and recovery automatically.

#### `boolean isExecutionComplete()`
Returns `true` if all commands completed successfully, `false` if any failed.

#### `void clearSteps()`
Clears all queued commands without executing.

#### `int getStepCount()`
Returns number of queued commands.

### Advanced Access

#### `AdvancedPositioningHelper getPositioningHelper()`
Provides direct access to the underlying positioning system for advanced control.

---

## 🎯 Advanced Positioning Helper (APH) API

### Constructor & Initialization

#### `AdvancedPositioningHelper(LinearOpMode opMode)`
Creates new positioning helper instance.

#### Initialization Methods
```java
// Basic camera initialization with auto AprilTag localization
void initialize(String webcamName)

// Full control initialization
void initialize(String webcamName, boolean attemptDeadWheels, boolean attemptAprilTagLocalization)

// Dual camera initialization (recommended)
void initializeDualCamera(String frontCameraName, String backCameraName,
                         double frontX, double frontY, double frontZ, double frontHeading,
                         double backX, double backY, double backZ, double backHeading)

// Dual camera with dead wheel control
void initializeDualCamera(String frontCameraName, String backCameraName,
                         double frontX, double frontY, double frontZ, double frontHeading,
                         double backX, double backY, double backZ, double backHeading,
                         boolean attemptDeadWheels, boolean attemptAprilTagLocalization)
```

### Position Control

#### `boolean goToPosition(double targetX, double targetY, double targetHeading, double maxSpeed)`
Moves robot to absolute field position with PID control.
- **Returns**: `true` if successful, `false` if failed/timeout
- **maxSpeed**: 0.0 to 1.0 (recommend 0.4-0.8)

#### `boolean rotateToHeading(double targetHeading, double maxSpeed)`
Rotates robot to absolute heading.
- **targetHeading**: Degrees (0° = facing Blue Alliance)
- **Returns**: `true` if successful

#### `void setDrivePowers(double forward, double strafe, double rotate)`
Direct motor control for advanced movement.
- **forward**: -1.0 to 1.0 (positive = toward audience)
- **strafe**: -1.0 to 1.0 (positive = toward Blue Alliance)  
- **rotate**: -1.0 to 1.0 (positive = counterclockwise)

#### `void stopMotors()`
Immediately stops all drive motors.

### Position Management

#### `void resetPosition(double x, double y, double heading)`
Sets robot position and resets all odometry systems.

#### `void setPosition(double x, double y, double heading)`
Updates current position without resetting odometry.

#### `boolean attemptAprilTagRelocalization()`
Attempts to correct position using visible AprilTags.
- **Returns**: `true` if successful correction made

### Position Queries

#### `double getCurrentX()`
Returns current X position in inches.

#### `double getCurrentY()`
Returns current Y position in inches.

#### `double getCurrentHeading()`
Returns current heading in degrees (0° = facing Blue Alliance).

#### `double[] getCurrentPosition()`
Returns `[x, y, heading]` array.

#### `double[] getVelocity()`
Returns `[vx, vy, omega]` velocity array (if supported by odometry system).

### System Information

#### `void updatePosition()`
Updates position from all available sensors. Called automatically during movement.

#### `void displayAprilTagInfo()`
Shows detailed AprilTag detection information in telemetry.

#### `DeadWheelOdometry getDeadWheelOdometry()`
Returns dead wheel odometry instance (null if not available).

#### `PinpointOdometryAdapter getPinpointOdometry()`
Returns Pinpoint odometry instance (null if not available).

### Utility Methods

#### `static double normalizeAngle(double angle)`
Normalizes angle to -180° to +180° range.

#### `static boolean isInRedAlliance(double x, double y)`
Returns `true` if position is in Red Alliance area.

#### `static boolean isInBlueAlliance(double x, double y)`
Returns `true` if position is in Blue Alliance area.

#### `static double getHeadingToPoint(double fromX, double fromY, double toX, double toY)`
Calculates heading from one point to another.

### Field Constants
```java
// Field boundaries
public static final double BACK_WALL_X = -70.5;      // Back wall (opposite audience)
public static final double AUDIENCE_WALL_X = 70.5;   // Front wall (audience side)
public static final double RED_WALL_Y = -70.5;       // Red Wall (left from audience)
public static final double BLUE_ALLIANCE_Y = 70.5;   // Blue Alliance (right from audience)

// Goal positions
public static final double RED_GOAL_X = -58.3727;    // Red Goal X
public static final double RED_GOAL_Y = 55.6425;     // Red Goal Y
public static final double BLUE_GOAL_X = -58.3727;   // Blue Goal X  
public static final double BLUE_GOAL_Y = -55.6425;   // Blue Goal Y

// Alliance starting areas
public static final double RED_ALLIANCE_X = -12.0;   // Red Alliance starting X
public static final double BLUE_ALLIANCE_X = 12.0;   // Blue Alliance starting X
```

---

## 🛣️ Movement Traversal Modes

The system provides 5 different movement traversal strategies to handle obstacles and optimize paths:

### `DIRECT_VECTOR` (Default)
- **Behavior**: Straight line movement to target
- **Pros**: Fastest, most direct path
- **Cons**: May hit obstacles
- **Best For**: Open field movement, when path is clear

### `X_THEN_Y`  
- **Behavior**: Move X first, then Y, then rotate
- **Pros**: Predictable L-shaped path, good for obstacle avoidance
- **Cons**: Longer path than direct
- **Best For**: Moving around field walls, avoiding central obstacles

### `Y_THEN_X`
- **Behavior**: Move Y first, then X, then rotate  
- **Pros**: Alternative L-shaped path
- **Cons**: Longer path than direct
- **Best For**: Alternative obstacle avoidance when X_THEN_Y won't work

### `MANHATTAN_AUTO`
- **Behavior**: Automatically chooses X_THEN_Y or Y_THEN_X based on larger distance
- **Pros**: Smart path selection, good general obstacle avoidance
- **Cons**: Less predictable than fixed modes
- **Best For**: General autonomous where you want smart path selection

### `SEPARATE_PHASES`
- **Behavior**: Pure X movement, then pure Y, then pure rotation (completely separate)
- **Pros**: Most predictable, easiest to debug
- **Cons**: Slowest, most mechanical looking
- **Best For**: Precision tasks, when you need exact control over each axis

### Usage Examples
```java
// Set default mode for all movements
autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.MANHATTAN_AUTO);

// Override mode for specific movement
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.X_THEN_Y, "Avoid center obstacle");

// Mix modes in same routine
autoHelper
    .moveTo(12, 60, 0, AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Quick move to pickup")
    .moveTo(48, 36, 90, AutoHelper.MovementTraversalMode.Y_THEN_X, "Navigate around samples")
    .executeAll();
```

---

## 🗺️ Field Coordinate System

The system uses the official **FTC DECODE Field Coordinate System**:

### Coordinate System
- **Origin (0,0,0)**: Field center, on the floor
- **X-axis**: Positive toward audience (front of field)
- **Y-axis**: Positive from Red Wall toward Blue Alliance  
- **Z-axis**: Positive vertically upward
- **Heading**: 0° = facing toward Blue Alliance (+Y direction)

### Field Layout
```
       Audience (Front, +X)
           ↑ +X
           │
-Y ←───────┼───────→ +Y
    Red    │    Blue
   Wall    │  Alliance
           │
      Back Wall (-X)
```

### Key Positions
```java
// Alliance starting positions  
double[] redStart = {-12.0, -52.5, 90.0};    // Red Alliance start
double[] blueStart = {12.0, 52.5, 270.0};    // Blue Alliance start

// Goal positions
double[] redGoal = {-58.3727, 55.6425, 315.0};   // Red Goal (back-right)
double[] blueGoal = {-58.3727, -55.6425, 45.0};  // Blue Goal (back-left)

// Field center
double[] center = {0.0, 0.0, 0.0};
```

### Common Autonomous Positions
```java
// Example Red Alliance autonomous path
autoHelper
    .moveTo(-12, -48, 0, "Move forward from start")      // Leave starting area
    .moveTo(0, -48, 45, "Turn toward center")            // Head to center
    .moveTo(24, -24, 90, "Approach scoring area")        // Move to scoring
    .moveTo(-58, 55, 315, "Score at Red Goal")           // Score at goal
    .executeAll();
```

---

## ⚙️ Configuration Examples

### Basic Motor Encoder Setup
```java
public class BasicAuto extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize();  // Motor encoders only
        
        // Set starting position manually
        autoHelper.getPositioningHelper().resetPosition(-12, -52.5, 90);
        
        waitForStart();
        // Your autonomous code here
    }
}
```

### Single Camera Setup
```java
public class CameraAuto extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1");  // Single camera + AprilTag localization
        
        // Configure for reliability
        autoHelper.setRelocalizationEnabled(true);
        autoHelper.setStuckDetectionEnabled(true);
        
        waitForStart();
        // Robot will auto-locate using AprilTags
    }
}
```

### Dual Camera Setup (Recommended)
```java
public class DualCameraAuto extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1", "Webcam 2");  // Dual cameras
        
        // Advanced configuration
        autoHelper.setRelocalizationEnabled(true);
        autoHelper.setStuckDetectionEnabled(true);
        autoHelper.setDebugTelemetryEnabled(true);
        autoHelper.setDefaultParameters(0.6, 5000);
        autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.MANHATTAN_AUTO);
        
        waitForStart();
        // Maximum AprilTag coverage and reliability
    }
}
```

### Dead Wheel Odometry Setup
```java
public class DeadWheelAuto extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        
        // Try dead wheels first, fallback to encoders+cameras
        autoHelper.initialize("Webcam 1", "Webcam 2", true);  // attemptDeadWheels = true
        
        waitForStart();
        // Will use dead wheels if available, otherwise encoders+cameras
    }
}
```

### Advanced Direct APH Setup
```java
public class AdvancedAuto extends LinearOpMode {
    private AdvancedPositioningHelper positioning;
    
    @Override
    public void runOpMode() {
        positioning = new AdvancedPositioningHelper(this);
        
        // Dual camera with specific positioning
        positioning.initializeDualCamera(
            "Webcam 1", "Webcam 2",           // Camera names
            6.0, 0.0, 8.0, 0.0,              // Front camera: 6" forward, 8" up, facing forward
            -6.0, 0.0, 8.0, 180.0,           // Back camera: 6" back, 8" up, facing backward
            true, true                       // Try dead wheels, enable auto-localization
        );
        
        waitForStart();
        
        // Direct positioning control
        positioning.goToPosition(24, 36, 90, 0.6);
        positioning.rotateToHeading(45, 0.4);
    }
}
```

---

## 🔧 Troubleshooting

### Common Issues

#### Robot Not Moving
1. **Check motor configuration**: Ensure motors are named correctly in hardware map
2. **Check encoder connections**: Verify encoder cables are connected
3. **Check power levels**: Default 60% power, increase if needed
4. **Check timeouts**: Default 5 seconds, increase for long movements

#### Inaccurate Positioning  
1. **Verify wheel diameter**: Check `WHEEL_DIAMETER_INCHES` constant
2. **Verify gear ratios**: Check `DRIVE_GEAR_REDUCTION` constant  
3. **Calibrate dead wheels**: Use `DeadWheelTuning.java` OpMode
4. **Check track width**: Measure actual robot track width

#### AprilTag Issues
1. **Camera positioning**: Ensure cameras have clear view of field
2. **Lighting conditions**: Avoid bright lights shining into cameras
3. **Tag visibility**: Ensure AprilTags are not obstructed
4. **Camera focus**: Check camera focus is set correctly

#### Movement Gets Stuck
1. **Lower power**: Reduce default power (try 0.4-0.5)
2. **Increase stuck threshold**: Increase stuck detection distance
3. **Check traversal mode**: Try `X_THEN_Y` or `Y_THEN_X` for obstacles
4. **Manual recovery**: Use custom actions for complex situations

### Debug Telemetry

Enable debug telemetry for detailed information:
```java
autoHelper.setDebugTelemetryEnabled(true);
positioning.displayAprilTagInfo();  // Shows AprilTag detection details
```

### Performance Tuning

#### For Speed
```java
autoHelper.setDefaultParameters(0.8, 3000);  // Higher power, shorter timeout
autoHelper.setDefaultTraversalMode(DIRECT_VECTOR);  // Fastest paths
```

#### For Accuracy  
```java
autoHelper.setDefaultParameters(0.4, 7000);  // Lower power, longer timeout
autoHelper.setRelocalizationEnabled(true);   // More AprilTag corrections
```

#### For Reliability
```java
autoHelper.setDefaultParameters(0.6, 5000);  // Balanced power/timeout
autoHelper.setStuckDetectionEnabled(true);   // Handle obstacles
autoHelper.setDefaultTraversalMode(MANHATTAN_AUTO);  // Smart obstacle avoidance
```

---

## 📝 Sample OpModes

The system includes comprehensive sample OpModes in the `samples/` package:

### Templates (`samples/templates/`)
- **`AutoTemplate.java`**: Comprehensive template with all features
- **`BlankAutoTemplate.java`**: Minimal starting template
- **`SimpleAutoTemplate.java`**: Basic autonomous template
- **`PrecisionAutoTemplate.java`**: Advanced precision template

### Examples (`samples/autonomous/`)
- **`ExampleAutoWithHelper.java`**: Full AutoHelper demonstration
- **`TraversalModesDemo.java`**: All 5 traversal modes demonstration
- **`AutoHelperQuickTest.java`**: Quick functionality test
- **`AdvancedPositioningDemo.java`**: Direct APH usage
- **`DeadWheelOdometryDemo.java`**: Dead wheel system demo
- **`UnifiedOdometryDemo.java`**: Unified odometry demo
- **`FTCCoordinateSystemDemo.java`**: Field coordinate system demo

### Usage
1. **Copy** a template from `samples/templates/` to create your autonomous
2. **Rename** the class and file to match your autonomous name
3. **Modify** the autonomous steps for your strategy
4. **Reference** examples in `samples/autonomous/` for advanced techniques

---

## 🎯 Best Practices

### Competition Setup
```java  
// Recommended competition configuration
autoHelper.setRelocalizationEnabled(true);      // Use AprilTags for corrections
autoHelper.setStuckDetectionEnabled(true);      // Handle unexpected obstacles  
autoHelper.setDebugTelemetryEnabled(false);     // Disable debug for competition
autoHelper.setDefaultParameters(0.6, 5000);     // Balanced speed/reliability
autoHelper.setDefaultTraversalMode(MANHATTAN_AUTO);  // Smart obstacle avoidance
```

### Development/Tuning Setup
```java
// Recommended development configuration  
autoHelper.setDebugTelemetryEnabled(true);      // Show detailed debugging
autoHelper.setDefaultParameters(0.4, 7000);     // Slower, more time for observation
positioning.displayAprilTagInfo();              // Show AprilTag detection details
```

### Error Handling
```java
// Always check execution results
autoHelper.executeAll();
if (!autoHelper.isExecutionComplete()) {
    telemetry.addData("Status", "Autonomous FAILED - check telemetry");
    // Add recovery actions here
} else {
    telemetry.addData("Status", "Autonomous completed successfully!");
}
```

---

## 📚 Additional Resources

- **Tuning OpMode**: Use `DeadWheelTuning.java` to calibrate odometry
- **Coordinate System**: Reference `FTCCoordinateSystemDemo.java` for field layout
- **AprilTag Setup**: See `AdvancedPositioningDemo.java` for camera configuration
- **Advanced Usage**: Check `ExampleAutoWithHelper.java` for comprehensive examples

For support and updates, check the code documentation and sample OpModes included with this system.

---

**System Version**: APH v2.1 / AAF v3.0  
**Last Updated**: September 2025  
**Compatibility**: FTC SDK 9.0+, DECODE Game Season