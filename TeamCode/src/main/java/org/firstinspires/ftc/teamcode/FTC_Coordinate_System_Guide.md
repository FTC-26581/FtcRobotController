# FTC Field Coordinate System Guide

## Overview
This guide explains the official FTC Field Coordinate System implementation in our autonomous code framework. The system ensures consistency with FTC standards and competition requirements.

## FTC Official Coordinate System

### Field Layout
- **Field Size**: 144" × 144" (12' × 12') square
- **Origin**: Center of the field (0, 0)
- **Coordinate Range**: -72" to +72" in both X and Y directions

### Axis Definitions
- **X-Axis**: 
  - Positive X (+X): Blue Alliance Wall (right side when viewed from audience)
  - Negative X (-X): Red Alliance Wall (left side when viewed from audience)
  - Range: -72" to +72"

- **Y-Axis**:
  - Positive Y (+Y): Away from Audience (far side of field)
  - Negative Y (-Y): Audience Side (near side of field)
  - Range: -72" to +72"

### Heading System
- **0°**: Robot faces positive Y direction (away from audience)
- **90°**: Robot faces positive X direction (toward Blue Alliance Wall)
- **180°**: Robot faces negative Y direction (toward audience)
- **270°**: Robot faces negative X direction (toward Red Alliance Wall)
- **Rotation**: Clockwise positive (standard mathematical convention inverted)

### Alliance Areas
- **Red Alliance**: Typically positioned at negative X values (left side)
- **Blue Alliance**: Typically positioned at positive X values (right side)

## Code Implementation

### AdvancedPositioningHelper Class
The `AdvancedPositioningHelper` class implements the FTC coordinate system with:

#### Key Constants
```java
public static final double FIELD_CENTER_TO_WALL = 72.0; // inches
public static final double FIELD_WIDTH = 144.0; // inches
public static final double FIELD_LENGTH = 144.0; // inches
public static final double RED_ALLIANCE_X = -72.0; // Red Wall X coordinate
public static final double BLUE_ALLIANCE_X = 72.0; // Blue Wall X coordinate
public static final double AUDIENCE_Y = -72.0; // Audience side Y coordinate
public static final double AWAY_FROM_AUDIENCE_Y = 72.0; // Away from audience Y coordinate
```

#### Helper Functions
- `fieldAngleToHeading(double fieldAngle)`: Convert field angles to robot headings
- `getHeadingToPoint(x1, y1, x2, y2)`: Calculate heading to face another point
- `isInRedAlliance(x, y)`: Check if position is in Red Alliance area
- `isInBlueAlliance(x, y)`: Check if position is in Blue Alliance area
- `getDistanceFromRedAlliance(x, y)`: Distance from Red Alliance area
- `getDistanceFromBlueAlliance(x, y)`: Distance from Blue Alliance area

### AutoHelper Integration
The `AutoHelper` class works seamlessly with the FTC coordinate system:

```java
// Move to specific field position
autoHelper.moveToPosition(36.0, -24.0, 90.0, power, accuracy, timeout);

// Turn to face specific field direction
autoHelper.turnToHeading(0.0, power, timeout); // Face away from audience
```

## Usage Examples

### Starting Positions
```java
// Red Alliance starting position (example)
positioning.setPosition(-60.0, -60.0, 0.0); // Near Red Wall, away from audience, facing away

// Blue Alliance starting position (example)  
positioning.setPosition(60.0, -60.0, 0.0); // Near Blue Wall, away from audience, facing away
```

### Common Field Positions
```java
// Field center
double[] centerPosition = {0.0, 0.0, 0.0};

// Red Alliance Wall center
double[] redWallCenter = {-72.0, 0.0, 90.0}; // At Red Wall, facing Blue Alliance

// Blue Alliance Wall center
double[] blueWallCenter = {72.0, 0.0, 270.0}; // At Blue Wall, facing Red Alliance

// Audience side center
double[] audienceCenter = {0.0, -72.0, 180.0}; // At audience side, facing away

// Away from audience center
double[] awayCenter = {0.0, 72.0, 0.0}; // Away from audience, facing away
```

### Navigation Examples
```java
// Move from Red Alliance to Blue Alliance
autoHelper.moveToPosition(-60.0, 0.0, 0.0, 0.8, 2.0, 5.0); // Start at Red side
autoHelper.moveToPosition(60.0, 0.0, 0.0, 0.8, 2.0, 5.0);  // Move to Blue side

// Navigate around field perimeter
autoHelper.moveToPosition(-60.0, -60.0, 0.0, 0.8, 2.0, 5.0); // Red Alliance, audience side
autoHelper.moveToPosition(60.0, -60.0, 90.0, 0.8, 2.0, 5.0); // Blue Alliance, audience side
autoHelper.moveToPosition(60.0, 60.0, 180.0, 0.8, 2.0, 5.0); // Blue Alliance, away side
autoHelper.moveToPosition(-60.0, 60.0, 270.0, 0.8, 2.0, 5.0); // Red Alliance, away side
```

## Best Practices

### 1. Alliance-Specific Code
```java
boolean isRedAlliance = true; // Set based on alliance selection

if (isRedAlliance) {
    // Red Alliance autonomous sequence
    positioning.setPosition(-60.0, -60.0, 0.0);
    // ... Red Alliance specific movements
} else {
    // Blue Alliance autonomous sequence  
    positioning.setPosition(60.0, -60.0, 0.0);
    // ... Blue Alliance specific movements
}
```

### 2. Field Element Positioning
Use consistent coordinate naming for field elements:
```java
// Example field elements (adjust for specific game)
public static final double[] CENTRAL_GOAL = {0.0, 0.0};
public static final double[] RED_DEPOT = {-48.0, -48.0};
public static final double[] BLUE_DEPOT = {48.0, -48.0};
public static final double[] AUDIENCE_BARRIER = {0.0, -60.0};
```

### 3. Heading Management
Always use field-relative headings:
```java
// Face toward opponent alliance
double headingToOpponent = isRedAlliance ? 90.0 : 270.0;
autoHelper.turnToHeading(headingToOpponent, 0.5, 3.0);

// Face toward audience
autoHelper.turnToHeading(180.0, 0.5, 3.0);

// Face away from audience  
autoHelper.turnToHeading(0.0, 0.5, 3.0);
```

### 4. Position Validation
```java
double[] currentPos = positioning.getPosition();

// Validate position is within field bounds
if (Math.abs(currentPos[0]) > 72.0 || Math.abs(currentPos[1]) > 72.0) {
    telemetry.addData("Warning", "Position outside field bounds!");
}

// Check alliance area
if (AdvancedPositioningHelper.isInRedAlliance(currentPos[0], currentPos[1])) {
    telemetry.addData("Location", "Red Alliance Area");
} else if (AdvancedPositioningHelper.isInBlueAlliance(currentPos[0], currentPos[1])) {
    telemetry.addData("Location", "Blue Alliance Area");
} else {
    telemetry.addData("Location", "Neutral Field Area");
}
```

## Sensor Fusion Details

The positioning system combines multiple sensors for accuracy:

1. **Encoder Odometry**: Primary position tracking using wheel encoders
2. **IMU Gyroscope**: Heading corrections and rotational tracking  
3. **AprilTag Vision**: Absolute position corrections using field markers

### Coordinate Transformations
All sensor data is automatically converted to FTC field coordinates:
- Encoder positions are transformed from robot-relative to field-relative
- IMU headings are adjusted to FTC standard (0° = positive Y direction)
- AprilTag positions are converted from camera coordinates to field coordinates

## Competition Compliance

This implementation ensures:
- ✅ Official FTC coordinate system compliance
- ✅ Consistent field positioning across different robots
- ✅ Standard heading conventions for drive team clarity
- ✅ Alliance-neutral code with proper transformations
- ✅ Integration with FTC SDK and hardware

## Troubleshooting

### Common Issues
1. **Inverted Coordinates**: Ensure sensors are properly calibrated to FTC standards
2. **Heading Drift**: Regular IMU calibration and AprilTag corrections help maintain accuracy
3. **Position Jumping**: Check for interference between sensor fusion components

### Debug Tools
```java
// Enable detailed positioning telemetry
positioning.setDebugMode(true);

// Display current coordinate system info
telemetry.addData("Coordinate System", "FTC Official");
telemetry.addData("Position", positioning.getFormattedPosition());
telemetry.addData("Quality", positioning.getPositionQuality());
```

## Competition Templates

See the following example OpModes for complete implementations:
- `FTCCoordinateSystemDemo.java`: Comprehensive coordinate system demonstration
- `PrecisionAutoTemplate.java`: Competition-ready template with precision positioning
- `BlankAutoTemplate.java`: Configurable template with multiple drive modes

This coordinate system implementation provides the foundation for consistent, reliable autonomous programming that meets FTC competition standards.