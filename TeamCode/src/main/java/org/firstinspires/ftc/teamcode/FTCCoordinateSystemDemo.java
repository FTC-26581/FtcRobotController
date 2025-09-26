package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Demonstration OpMode showcasing the FTC Field Coordinate System
 * 
 * This OpMode demonstrates:
 * 1. Proper FTC coordinate system usage
 * 2. Movement to specific field positions
 * 3. Heading management using FTC standards
 * 4. Alliance-specific positioning
 * 
 * FTC Field Coordinate System Summary:
 * - Origin (0,0) at field center
 * - X-axis: Blue Alliance Wall (+X) to Red Alliance Wall (-X)
 * - Y-axis: Audience Side (+Y) to Away from Audience (-Y)  
 * - Heading: 0° faces positive Y (away from audience), increases clockwise
 * - Field dimensions: 144" x 144" (12' x 12')
 */
@Autonomous(name = "FTC Coordinate System Demo", group = "Demo")
public class FTCCoordinateSystemDemo extends OpMode {
    
    private AutoHelper autoHelper;
    private AdvancedPositioningHelper positioning;
    private boolean allianceIsRed = true; // Set based on alliance selection
    
    // Example field positions using FTC coordinate system
    private static final double[][] RED_ALLIANCE_POSITIONS = {
        {-60.0, -60.0, 0.0},   // Near Red Wall, away from audience
        {-60.0, 0.0, 90.0},    // Red Wall, field center, face Blue Alliance
        {-60.0, 60.0, 180.0},  // Red Wall, audience side, face audience
        {0.0, 0.0, 0.0}        // Field center, face away from audience
    };
    
    private static final double[][] BLUE_ALLIANCE_POSITIONS = {
        {60.0, -60.0, 0.0},    // Near Blue Wall, away from audience  
        {60.0, 0.0, 270.0},    // Blue Wall, field center, face Red Alliance
        {60.0, 60.0, 180.0},   // Blue Wall, audience side, face audience
        {0.0, 0.0, 0.0}        // Field center, face away from audience
    };
    
    @Override
    public void init() {
        // Initialize helpers
        autoHelper = new AutoHelper(this, telemetry);
        positioning = new AdvancedPositioningHelper(hardwareMap, telemetry, autoHelper.getGyro());
        
        // Configure for precision positioning
        autoHelper.setPrecisionPositioning(true, positioning);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Coordinate System", "FTC Official");
        telemetry.addData("Alliance", allianceIsRed ? "Red" : "Blue");
        telemetry.addData("Instructions", "Press start to run coordinate demo");
        telemetry.update();
    }
    
    @Override
    public void start() {
        autoHelper.start();
        
        // Set initial position based on alliance
        if (allianceIsRed) {
            // Red Alliance typically starts at negative X (left side when viewed from audience)
            positioning.setPosition(-60.0, -60.0, 0.0); // Near Red Wall, away from audience, facing away
        } else {
            // Blue Alliance typically starts at positive X (right side when viewed from audience)  
            positioning.setPosition(60.0, -60.0, 0.0); // Near Blue Wall, away from audience, facing away
        }
        
        telemetry.addData("Status", "Demo Started");
        telemetry.addData("Starting Position", positioning.getFormattedPosition());
        telemetry.update();
    }
    
    @Override
    public void loop() {
        autoHelper.loop();
        
        switch (autoHelper.getCurrentStep()) {
            case 1:
                autoHelper.addTelemetry("Step", "Demonstrating FTC Coordinate System");
                autoHelper.addTelemetry("Current Pos", positioning.getFormattedPosition());
                
                // Show current position relative to field elements
                double[] pos = positioning.getPosition();
                autoHelper.addTelemetry("Distance to Center", String.format("%.1f\"", 
                    Math.sqrt(pos[0]*pos[0] + pos[1]*pos[1])));
                autoHelper.addTelemetry("Distance to Red Wall", String.format("%.1f\"", 
                    Math.abs(pos[0] - AdvancedPositioningHelper.RED_ALLIANCE_X)));
                autoHelper.addTelemetry("Distance to Blue Wall", String.format("%.1f\"", 
                    Math.abs(pos[0] - AdvancedPositioningHelper.BLUE_ALLIANCE_X)));
                
                autoHelper.setStepComplete();
                break;
                
            case 2:
                // Move to field center
                autoHelper.addTelemetry("Step", "Moving to Field Center");
                autoHelper.moveToPosition(0.0, 0.0, 0.0, 0.8, 2.0, 5.0);
                break;
                
            case 3:
                // Demonstrate heading changes at center
                autoHelper.addTelemetry("Step", "Demonstrating Headings at Center");
                autoHelper.addTelemetry("Info", "0°=Away from audience, 90°=Blue Wall");
                
                // Face each direction for 2 seconds each
                if (autoHelper.getStepTime() < 2.0) {
                    autoHelper.turnToHeading(0.0, 0.5, 3.0); // Face away from audience
                    autoHelper.addTelemetry("Facing", "Away from Audience (0°)");
                } else if (autoHelper.getStepTime() < 4.0) {
                    autoHelper.turnToHeading(90.0, 0.5, 3.0); // Face Blue Alliance
                    autoHelper.addTelemetry("Facing", "Blue Alliance Wall (90°)");
                } else if (autoHelper.getStepTime() < 6.0) {
                    autoHelper.turnToHeading(180.0, 0.5, 3.0); // Face audience
                    autoHelper.addTelemetry("Facing", "Audience Side (180°)");
                } else if (autoHelper.getStepTime() < 8.0) {
                    autoHelper.turnToHeading(270.0, 0.5, 3.0); // Face Red Alliance
                    autoHelper.addTelemetry("Facing", "Red Alliance Wall (270°)");
                } else {
                    autoHelper.setStepComplete();
                }
                break;
                
            case 4:
                // Move to alliance-specific positions
                autoHelper.addTelemetry("Step", "Moving to Alliance Positions");
                
                double[][] positions = allianceIsRed ? RED_ALLIANCE_POSITIONS : BLUE_ALLIANCE_POSITIONS;
                int posIndex = (int)(autoHelper.getStepTime() / 3.0); // Change position every 3 seconds
                
                if (posIndex < positions.length) {
                    double[] targetPos = positions[posIndex];
                    autoHelper.moveToPosition(targetPos[0], targetPos[1], targetPos[2], 0.8, 2.0, 5.0);
                    autoHelper.addTelemetry("Target", String.format("X:%.1f Y:%.1f H:%.0f°", 
                        targetPos[0], targetPos[1], targetPos[2]));
                } else {
                    autoHelper.setStepComplete();
                }
                break;
                
            case 5:
                // Demonstrate position reporting
                autoHelper.addTelemetry("Step", "Position Reporting Demo");
                double[] currentPos = positioning.getPosition();
                
                autoHelper.addTelemetry("Position", positioning.getFormattedPosition());
                autoHelper.addTelemetry("In Red Alliance", 
                    AdvancedPositioningHelper.isInRedAlliance(currentPos[0], currentPos[1]) ? "YES" : "NO");
                autoHelper.addTelemetry("In Blue Alliance", 
                    AdvancedPositioningHelper.isInBlueAlliance(currentPos[0], currentPos[1]) ? "YES" : "NO");
                
                // Calculate heading to opposite alliance
                if (allianceIsRed) {
                    double headingToBlue = AdvancedPositioningHelper.getHeadingToPoint(
                        currentPos[0], currentPos[1], 60.0, 0.0);
                    autoHelper.addTelemetry("Heading to Blue Alliance", String.format("%.1f°", headingToBlue));
                } else {
                    double headingToRed = AdvancedPositioningHelper.getHeadingToPoint(
                        currentPos[0], currentPos[1], -60.0, 0.0);
                    autoHelper.addTelemetry("Heading to Red Alliance", String.format("%.1f°", headingToRed));
                }
                
                if (autoHelper.getStepTime() > 5.0) {
                    autoHelper.setStepComplete();
                }
                break;
                
            case 6:
                // Final position - return to start
                autoHelper.addTelemetry("Step", "Returning to Start Position");
                if (allianceIsRed) {
                    autoHelper.moveToPosition(-60.0, -60.0, 0.0, 0.8, 2.0, 5.0);
                } else {
                    autoHelper.moveToPosition(60.0, -60.0, 0.0, 0.8, 2.0, 5.0);
                }
                break;
                
            case 7:
                autoHelper.addTelemetry("Status", "Demo Complete!");
                autoHelper.addTelemetry("Final Position", positioning.getFormattedPosition());
                autoHelper.addTelemetry("Coordinate System", "FTC Official Standard");
                
                // Stop all motors
                autoHelper.stopMotors();
                
                if (autoHelper.getStepTime() > 3.0) {
                    autoHelper.setStepComplete();
                }
                break;
                
            default:
                // Demo finished
                autoHelper.addTelemetry("Status", "Demo Finished - FTC Coordinates Demonstrated");
                autoHelper.stopMotors();
                break;
        }
        
        // Always show positioning data
        autoHelper.addTelemetry("Position Data", positioning.getFormattedPosition());
        autoHelper.addTelemetry("Positioning Quality", positioning.getPositionQuality());
        
        autoHelper.updateTelemetry();
    }
    
    @Override
    public void stop() {
        if (autoHelper != null) {
            autoHelper.stop();
        }
        if (positioning != null) {
            positioning.close();
        }
    }
}