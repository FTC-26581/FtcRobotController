package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Advanced Positioning Demo - Precision Movement Example
 * 
 * This OpMode demonstrates the advanced positioning system that combines:
 * - Encoder odometry for dead reckoning
 * - IMU gyroscope for heading accuracy
 * - AprilTag vision for absolute position correction
 * 
 * The robot will move to precise field coordinates and orientations,
 * showcasing the accuracy of the sensor fusion system.
 * 
 * To use:
 * 1. Uncomment the @Autonomous annotation
 * 2. Ensure your robot has a camera configured as "Webcam 1"
 * 3. Place AprilTags on the field according to your field setup
 * 4. Run the OpMode and observe precise movements
 */

// @Autonomous(name = "Advanced Positioning Demo", group = "Demo")
public class AdvancedPositioningDemo extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    // Field positions (adjust for your field setup)
    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;
    
    private static final double POSITION_1_X = 24.0;
    private static final double POSITION_1_Y = 12.0;
    private static final double POSITION_1_HEADING = 90.0;
    
    private static final double POSITION_2_X = -18.0;
    private static final double POSITION_2_Y = 36.0;
    private static final double POSITION_2_HEADING = 180.0;
    
    private static final double POSITION_3_X = 0.0;
    private static final double POSITION_3_Y = 48.0;
    private static final double POSITION_3_HEADING = 270.0;
    
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {
        
        // Initialize AutoHelper with advanced positioning
        autoHelper = new AutoHelper(this);
        autoHelper.initialize(AutoHelper.DriveMode.ADVANCED_POSITIONING, "Webcam 1");
        
        // Set starting position (calibrate based on where you place the robot)
        autoHelper.setCurrentPosition(START_X, START_Y, START_HEADING);
        
        telemetry.addData("Status", "Advanced Positioning Initialized");
        telemetry.addData("Starting Position", "X:%.1f Y:%.1f H:%.1f°", START_X, START_Y, START_HEADING);
        telemetry.addData("", "Place robot at starting position and press play");
        telemetry.update();
        
        waitForStart();
        autoHelper.resetRuntime();
        
        // Main autonomous sequence
        while (opModeIsActive()) {
            
            // Update position tracking
            autoHelper.updatePosition();
            
            // Execute current step
            executeStep(autoHelper.getCurrentStep());
            
            // Update telemetry
            autoHelper.updateTelemetry();
            
            // Check if autonomous should end
            if (autoHelper.getCurrentStep() >= getTotalSteps()) {
                autoHelper.addTelemetry("Status", "Demo Complete!");
                autoHelper.updateTelemetry();
                break;
            }
        }
        
        // Clean up resources
        autoHelper.close();
    }
    
    /**
     * Execute autonomous steps
     */
    private void executeStep(int step) {
        switch (step) {
            case 0:
                // Initial calibration wait
                if (autoHelper.waitFor(2.0)) {
                    autoHelper.addTelemetry("Step 0", "Calibration complete");
                    
                    // If AprilTag is visible, calibrate position
                    if (autoHelper.hasAprilTagFix()) {
                        autoHelper.addTelemetry("Calibration", "AprilTag detected - position corrected");
                    }
                }
                break;
                
            case 1:
                // Move to Position 1 with specific heading
                if (autoHelper.goToPosition(POSITION_1_X, POSITION_1_Y, POSITION_1_HEADING, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 1", "Reached Position 1");
                    autoHelper.addTelemetry("Target", "X:%.1f Y:%.1f H:%.1f°", POSITION_1_X, POSITION_1_Y, POSITION_1_HEADING);
                    autoHelper.addTelemetry("Actual", "X:%.1f Y:%.1f H:%.1f°", 
                            autoHelper.getCurrentX(), autoHelper.getCurrentY(), autoHelper.getCurrentHeading());
                }
                break;
                
            case 2:
                // Wait and show position accuracy
                if (autoHelper.waitFor(2.0)) {
                    double distanceError = autoHelper.getDistanceToTarget(POSITION_1_X, POSITION_1_Y);
                    autoHelper.addTelemetry("Step 2", "Position accuracy check");
                    autoHelper.addTelemetry("Distance Error", "%.2f inches", distanceError);
                }
                break;
                
            case 3:
                // Move to Position 2
                if (autoHelper.goToPosition(POSITION_2_X, POSITION_2_Y, POSITION_2_HEADING, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 3", "Reached Position 2");
                    autoHelper.addTelemetry("Target", "X:%.1f Y:%.1f H:%.1f°", POSITION_2_X, POSITION_2_Y, POSITION_2_HEADING);
                    autoHelper.addTelemetry("Actual", "X:%.1f Y:%.1f H:%.1f°", 
                            autoHelper.getCurrentX(), autoHelper.getCurrentY(), autoHelper.getCurrentHeading());
                }
                break;
                
            case 4:
                // Demonstration of pure rotation
                if (autoHelper.rotateToHeading(POSITION_2_HEADING + 180, TURN_SPEED)) {
                    autoHelper.addTelemetry("Step 4", "180° rotation complete");
                    autoHelper.addTelemetry("Target Heading", "%.1f°", POSITION_2_HEADING + 180);
                    autoHelper.addTelemetry("Actual Heading", "%.1f°", autoHelper.getCurrentHeading());
                }
                break;
                
            case 5:
                // Move to Position 3 (no heading change)
                if (autoHelper.goToPosition(POSITION_3_X, POSITION_3_Y, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 5", "Reached Position 3 (no heading change)");
                    autoHelper.addTelemetry("Target", "X:%.1f Y:%.1f", POSITION_3_X, POSITION_3_Y);
                    autoHelper.addTelemetry("Actual", "X:%.1f Y:%.1f", 
                            autoHelper.getCurrentX(), autoHelper.getCurrentY());
                }
                break;
                
            case 6:
                // Return to start position with original heading
                if (autoHelper.goToPosition(START_X, START_Y, START_HEADING, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 6", "Returned to start");
                    double finalDistanceError = autoHelper.getDistanceToTarget(START_X, START_Y);
                    autoHelper.addTelemetry("Final Position Error", "%.2f inches", finalDistanceError);
                }
                break;
                
            case 7:
                // Final accuracy assessment
                if (autoHelper.waitFor(3.0)) {
                    autoHelper.addTelemetry("Step 7", "Demo complete - Final accuracy assessment");
                    autoHelper.addTelemetry("Expected", "X:%.1f Y:%.1f H:%.1f°", START_X, START_Y, START_HEADING);
                    autoHelper.addTelemetry("Actual", "X:%.1f Y:%.1f H:%.1f°", 
                            autoHelper.getCurrentX(), autoHelper.getCurrentY(), autoHelper.getCurrentHeading());
                    
                    // Calculate total error
                    double xError = Math.abs(autoHelper.getCurrentX() - START_X);
                    double yError = Math.abs(autoHelper.getCurrentY() - START_Y);
                    double headingError = Math.abs(autoHelper.getCurrentHeading() - START_HEADING);
                    
                    autoHelper.addTelemetry("Position Errors", "X:%.2f Y:%.2f H:%.2f°", xError, yError, headingError);
                    autoHelper.addTelemetry("AprilTag Status", autoHelper.hasAprilTagFix() ? "ACTIVE" : "INACTIVE");
                }
                break;
                
            default:
                // End of demo
                autoHelper.nextStep();
                break;
        }
    }
    
    /**
     * Get total number of steps
     */
    private int getTotalSteps() {
        return 8;
    }
}