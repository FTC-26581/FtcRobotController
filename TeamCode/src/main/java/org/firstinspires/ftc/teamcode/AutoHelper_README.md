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
```

## Available Methods

### Movement (Fluent API)

- `moveTo(x, y, heading, description)` - Move to absolute position
- `moveBy(deltaX, deltaY, deltaHeading, description)` - Relative movement
- `turnTo(heading, description)` - Turn to absolute heading
- `turnBy(deltaHeading, description)` - Turn by relative angle

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

## Best Practices

1. **Always use descriptions**: Make your autonomous readable
2. **Enable recovery for competition**: Use `executeAllWithRecovery()`
3. **Test timeouts**: Adjust default parameters for your robot
4. **Use dual cameras**: Provides maximum AprilTag visibility
5. **Check results**: Always verify execution status
6. **Keep steps atomic**: Each step should do one clear thing

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