package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Precision Auto Template - Using Advanced Positioning
 * 
 * This template shows how to use the advanced positioning system for
 * precise autonomous movements in a real competition scenario.
 * 
 * Features demonstrated:
 * - Precise movement to scoring positions
 * - Accurate alignment for game piece pickup
 * - Return to exact starting position
 * - Sensor fusion for maximum accuracy
 * 
 * To customize for your robot:
 * 1. Uncomment the @Autonomous annotation
 * 2. Update field coordinates for your game
 * 3. Add game-specific actions between movements
 */

// @Autonomous(name = "Precision Auto Template", group = "Template")  
public class PrecisionAutoTemplate extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    // Game field coordinates (adjust for your competition)
    private static final double START_X = -36.0;      // Starting position X
    private static final double START_Y = -60.0;      // Starting position Y  
    private static final double START_HEADING = 90.0; // Facing forward
    
    private static final double PICKUP_X = 0.0;       // Game piece pickup X
    private static final double PICKUP_Y = -48.0;     // Game piece pickup Y
    private static final double PICKUP_HEADING = 0.0; // Facing pickup
    
    private static final double SCORING_X = 36.0;     // Scoring position X
    private static final double SCORING_Y = -60.0;    // Scoring position Y
    private static final double SCORING_HEADING = 90.0; // Facing scoring area
    
    private static final double PARK_X = -36.0;       // Park position X
    private static final double PARK_Y = -12.0;       // Park position Y
    private static final double PARK_HEADING = 0.0;   // Park orientation
    
    // Movement speeds
    private static final double FAST_SPEED = 0.8;
    private static final double PRECISION_SPEED = 0.4;
    private static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        
        // Initialize with advanced positioning and camera
        autoHelper = new AutoHelper(this);
        autoHelper.initialize(AutoHelper.DriveMode.ADVANCED_POSITIONING, "Webcam 1");
        
        // Set starting position (place robot precisely at these coordinates)
        autoHelper.setCurrentPosition(START_X, START_Y, START_HEADING);
        
        telemetry.addData("Status", "Precision Auto Ready");
        telemetry.addData("Starting Position", "X:%.1f Y:%.1f H:%.1f°", START_X, START_Y, START_HEADING);
        telemetry.addData("Camera", "Webcam 1 - AprilTag positioning active");
        telemetry.update();
        
        waitForStart();
        autoHelper.resetRuntime();
        
        // Main autonomous sequence
        while (opModeIsActive()) {
            
            // Continuously update position tracking
            autoHelper.updatePosition();
            
            // Execute current step
            executeStep(autoHelper.getCurrentStep());
                
            // Update telemetry
            autoHelper.updateTelemetry();
            
            // Check completion
            if (autoHelper.getCurrentStep() >= getTotalSteps()) {
                break;
            }
        }
        
        // Clean up
        autoHelper.close();
    }
    
    /**
     * Main autonomous sequence
     */
    private void executeStep(int step) {
        switch (step) {
            case 0:
                // Initial positioning and calibration
                if (autoHelper.waitFor(1.0)) {
                    autoHelper.addTelemetry("Step 0", "System calibrated");
                    if (autoHelper.hasAprilTagFix()) {
                        autoHelper.addTelemetry("AprilTag", "Position corrected");
                    }
                }
                break;
                
            case 1:
                // Move to game piece pickup position
                if (autoHelper.goToPosition(PICKUP_X, PICKUP_Y, PICKUP_HEADING, FAST_SPEED)) {
                    autoHelper.addTelemetry("Step 1", "Reached pickup position");
                    
                    // TODO: Add your game piece pickup code here
                    // Example: activateIntake(), lowerArm(), etc.
                }
                break;
                
            case 2:
                // Precise alignment for pickup (if needed)
                if (autoHelper.goToPosition(PICKUP_X, PICKUP_Y - 2, PICKUP_HEADING, PRECISION_SPEED)) {
                    autoHelper.addTelemetry("Step 2", "Precise pickup alignment");
                    
                    // TODO: Add pickup actions here
                    // Example: waitForGamePieceDetected(), closeGripper(), etc.
                }
                break;
                
            case 3:
                // Move to scoring position
                if (autoHelper.goToPosition(SCORING_X, SCORING_Y, SCORING_HEADING, FAST_SPEED)) {
                    autoHelper.addTelemetry("Step 3", "Reached scoring position");
                    
                    // TODO: Add pre-scoring preparation here
                    // Example: raiseArm(), prepareScoring(), etc.
                }
                break;
                
            case 4:
                // Precise scoring alignment
                if (autoHelper.rotateToHeading(SCORING_HEADING, TURN_SPEED)) {
                    autoHelper.addTelemetry("Step 4", "Scoring alignment complete");
                    
                    // TODO: Add scoring actions here
                    // Example: extendArm(), releaseGamePiece(), etc.
                }
                break;
                
            case 5:
                // Wait for scoring to complete
                if (autoHelper.waitFor(2.0)) {
                    autoHelper.addTelemetry("Step 5", "Scoring complete");
                    
                    // TODO: Add post-scoring actions
                    // Example: retractArm(), resetScoringMechanism(), etc.
                }
                break;
                
            case 6:
                // Move to park position
                if (autoHelper.goToPosition(PARK_X, PARK_Y, PARK_HEADING, FAST_SPEED)) {
                    autoHelper.addTelemetry("Step 6", "Parked successfully");
                    
                    // Calculate final accuracy
                    double distanceError = autoHelper.getDistanceToTarget(PARK_X, PARK_Y);
                    autoHelper.addTelemetry("Final Accuracy", "%.2f inches", distanceError);
                }
                break;
                
            case 7:
                // Final status report
                if (autoHelper.waitFor(1.0)) {
                    autoHelper.addTelemetry("Status", "Autonomous Complete!");
                    autoHelper.addTelemetry("Total Time", "%.1f seconds", autoHelper.getRuntime());
                    autoHelper.addTelemetry("AprilTag Used", autoHelper.hasAprilTagFix() ? "YES" : "NO");
                }
                break;
                
            default:
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
    
    // ========================================
    // CUSTOM GAME-SPECIFIC FUNCTIONS
    // Add your robot's specific functions here
    // ========================================
    
    /**
     * Example: Activate intake mechanism
     */
    private boolean activateIntake() {
        // TODO: Implement your intake control
        // Example: intakeMotor.setPower(1.0);
        return autoHelper.waitFor(1.0); // Placeholder timing
    }
    
    /**
     * Example: Score game piece
     */
    private boolean scoreGamePiece() {
        // TODO: Implement your scoring mechanism
        // Example: scoringServo.setPosition(SCORE_POSITION);
        return autoHelper.waitFor(1.5); // Placeholder timing
    }
    
    /**
     * Example: Check if AprilTag correction is needed
     */
    private void checkPositionCorrection() {
        if (autoHelper.hasAprilTagFix()) {
            autoHelper.addTelemetry("Position Correction", "AprilTag active - high accuracy");
        } else {
            autoHelper.addTelemetry("Position Correction", "Using encoders + IMU only");
        }
    }
}