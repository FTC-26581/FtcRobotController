# AutoHelper Framework v3.0

The AutoHelper framework provides a comprehensive, easy-to-use system for creating autonomous OpModes using the AdvancedPositioningHelper (APH) system. It features intelligent error detection, recovery mechanisms, and both fluent API and step-based autonomous programming.

## Key Features

- **Fluent API**: Intuitive method chaining for readable autonomous programs
- **Smart Error Detection**: Automatic timeout and stuck detection
- **Recovery Mechanisms**: AprilTag-based relocalization when stuck
- **Dual Camera Support**: Automatic switching between front/back cameras
- **Comprehensive Telemetry**: Detailed debugging and status information
- **Future-Ready**: Designed for easy extension with collision avoidance

## Quick Start

### 1. Basic Usage (Fluent API)

```java
@Autonomous(name="My Auto", group="Autonomous")
public class MyAuto extends LinearOpMode {
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize(); // Uses dual cameras by default
        
        waitForStart();
        
        autoHelper
            .moveTo(24, 36, 90, "Move to scoring position")
            .waitFor(1000, "Wait for arm")
            .moveBy(6, 0, 0, "Approach target")
            .executeAll();
    }
}
```

### 2. Advanced Usage with Recovery

```java
autoHelper = new AutoHelper(this, hardwareMap, telemetry);
autoHelper.setRelocalizationEnabled(true);  // Enable AprilTag recovery
autoHelper.setStuckDetectionEnabled(true);  // Enable stuck detection
autoHelper.initialize("Webcam 1", "Webcam 2");

autoHelper
    .addStep("Custom action", () -> {
        // Your custom code here
        return true; // Return success/failure
    })
    .moveTo(48, 24, 45, "Drive to target")
    .executeAllWithRecovery(); // Uses advanced recovery
```

## Configuration Options

### Camera Setup

```java
// Dual cameras (recommended)
autoHelper.initialize("Webcam 1", "Webcam 2");

// Single camera
autoHelper.initialize("Webcam 1");

// Encoders only (no cameras)
autoHelper.initialize(null, null);
```

### Feature Configuration

```java
autoHelper.setRelocalizationEnabled(true);    // AprilTag recovery
autoHelper.setStuckDetectionEnabled(true);    // Stuck detection
autoHelper.setDebugTelemetryEnabled(true);    // Detailed telemetry
autoHelper.setDefaultParameters(0.7, 4000);   // Power & timeout
autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.X_THEN_Y); // Movement mode
```

## Available Methods

### Movement (Fluent API)

- `moveTo(x, y, heading, description)` - Move to absolute position (default traversal)
- `moveTo(x, y, heading, traversalMode, description)` - Move with specific traversal mode
- `moveBy(deltaX, deltaY, deltaHeading, description)` - Relative movement
- `turnTo(heading, description)` - Turn to absolute heading
- `turnBy(deltaHeading, description)` - Turn by relative angle

### Movement Traversal Modes

Choose how the robot moves to target positions:

- `DIRECT_VECTOR` - Straight line to target (fastest, default)
- `X_THEN_Y` - Move X first, then Y, then rotate (obstacle avoidance)
- `Y_THEN_X` - Move Y first, then X, then rotate (alternative avoidance)
- `MANHATTAN_AUTO` - Choose X or Y first based on larger distance
- `SEPARATE_PHASES` - Pure X, then pure Y, then rotate (most predictable)

### Timing and Conditions

- `waitFor(milliseconds, description)` - Wait for time
- `waitUntil(condition, description)` - Wait for condition
- `waitUntil(condition, timeout, description)` - Wait with timeout

### Custom Actions

- `addStep(description, action)` - Add custom step that returns boolean
- `addAction(description, runnable)` - Add simple action

### Execution

- `executeAll()` - Execute with basic error handling
- `executeAllWithRecovery()` - Execute with advanced recovery

## Error Handling

The AutoHelper automatically handles common autonomous problems:

### Stuck Detection
- Monitors robot position over time
- Detects when robot hasn't moved significantly
- Configurable thresholds for distance and time

### Recovery Mechanisms
- **AprilTag Relocalization**: Uses cameras to correct position when stuck
- **Timeout Handling**: Automatically handles movement timeouts
- **Step Retry**: Can retry failed steps after recovery

### Error Reporting
```java
if (autoHelper.hasExecutionFailed()) {
    String error = autoHelper.getLastErrorMessage();
    // Handle error
}

// Get detailed execution report
String report = autoHelper.getExecutionReport();
```

## Step Management

### Clear and Rebuild Steps
```java
autoHelper.clearSteps()
    .moveTo(12, 12, 0, "New starting position")
    .executeAll();
```

### Check Execution Status
```java
boolean complete = autoHelper.isExecutionComplete();
boolean failed = autoHelper.hasExecutionFailed();
String currentStep = autoHelper.getCurrentStepInfo();
```

## Direct APH Access

For advanced operations, you can access the AdvancedPositioningHelper directly:

```java
AdvancedPositioningHelper aph = autoHelper.getPositioningHelper();
double x = aph.getCurrentX();
double y = aph.getCurrentY();
double heading = aph.getCurrentHeading();

// Mix direct APH calls with AutoHelper steps
autoHelper.addStep("Custom APH operation", () -> {
    return aph.moveTo(x + 12, y - 6, heading + 45, 0.5, 3000);
});
```

## Movement Traversal Modes

The AutoHelper supports five different ways to move to target positions:

### 1. DIRECT_VECTOR (Default)
```java
autoHelper.moveTo(24, 36, 90, "Move to basket"); // Uses default mode
```
- **Behavior**: Straight line to target position
- **Pros**: Fastest movement, most efficient
- **Cons**: Can hit obstacles in path
- **Use when**: Open field areas, maximum speed needed

### 2. X_THEN_Y
```java
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.X_THEN_Y, "Avoid obstacle");
```
- **Behavior**: Move X coordinate first, then Y, then rotate
- **Pros**: Predictable L-shaped path, good obstacle avoidance
- **Cons**: Longer path than direct vector
- **Use when**: Need to go around obstacles, strafe then forward pattern

### 3. Y_THEN_X
```java
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.Y_THEN_X, "Forward then strafe");
```
- **Behavior**: Move Y coordinate first, then X, then rotate
- **Pros**: Forward/back first, then strafe pattern
- **Cons**: Longer path than direct vector
- **Use when**: Need to clear walls first, forward then strafe pattern

### 4. MANHATTAN_AUTO
```java
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.MANHATTAN_AUTO, "Smart L-path");
```
- **Behavior**: Automatically chooses X or Y first based on larger distance
- **Pros**: Minimizes total distance while avoiding obstacles
- **Cons**: Path not always predictable
- **Use when**: Want L-shaped movement but don't know which axis is larger

### 5. SEPARATE_PHASES
```java
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.SEPARATE_PHASES, "Pure axis moves");
```
- **Behavior**: Pure strafe, then pure forward/back, then pure rotation
- **Pros**: Most predictable, each axis independent, easy to debug
- **Cons**: Slowest method, most phases
- **Use when**: Need precise control, debugging complex movements

### Setting Default Mode
```java
// Set default mode for all movements
autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.X_THEN_Y);

// Individual movements can still override
autoHelper
    .moveTo(12, 12, 0, "Uses default X_THEN_Y")
    .moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Override to direct")
    .executeAll();
```

## Best Practices

1. **Always use descriptions**: Make your autonomous readable
2. **Enable recovery for competition**: Use `executeAllWithRecovery()`
3. **Test timeouts**: Adjust default parameters for your robot
4. **Use dual cameras**: Provides maximum AprilTag visibility
5. **Check results**: Always verify execution status
6. **Keep steps atomic**: Each step should do one clear thing
7. **Choose appropriate traversal modes**: Use DIRECT_VECTOR for speed, Manhattan modes for obstacle avoidance

## Troubleshooting

### Common Issues

**"Robot appears stuck"**
- Check stuck detection parameters
- Verify motors are working
- Consider enabling relocalization

**"AprilTag relocalization failed"**
- Check camera connections
- Verify AprilTag positions
- Ensure adequate lighting

**"Step timeout"**
- Increase timeout values
- Check for mechanical issues
- Verify target positions are reachable

### Debug Information

Enable debug telemetry for detailed information:
```java
autoHelper.setDebugTelemetryEnabled(true);
autoHelper.displayTelemetry(); // Call during execution
```

## Files Overview

- `AutoHelper.java` - Main framework class
- `ExampleAutoWithHelper.java` - Comprehensive examples
- `AutoTemplate.java` - Template for creating new autonomous OpModes
- `AdvancedPositioningHelper.java` - Core positioning system

## Migration from Old AutoHelper

The new AutoHelper is designed to be much simpler while being more powerful:

**Old way:**
```java
autoHelper.initialize(DriveMode.ADVANCED_POSITIONING, "Webcam 1");
// Complex step management
```

**New way:**
```java
autoHelper.initialize("Webcam 1", "Webcam 2");
autoHelper.moveTo(24, 36, 90, "Move to basket").executeAll();
```

## Future Enhancements

The AutoHelper framework is designed to easily accommodate future features:

- Collision avoidance using distance sensors
- Path planning and optimization
- Multi-robot coordination
- Advanced sensor fusion
- Machine learning integration

---

For more examples and detailed usage, see `ExampleAutoWithHelper.java` and `AutoTemplate.java`.