package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

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
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1");

        telemetry.addData("Status", "Precision Auto Ready");
        telemetry.addData("Starting Position", "X:%.1f Y:%.1f H:%.1f°", START_X, START_Y, START_HEADING);
        telemetry.addData("Camera", "Webcam 1 - AprilTag positioning active");
        telemetry.update();
        
        waitForStart();

        // Execute the autonomous sequence using AutoHelper's fluent API
        if (opModeIsActive()) {
            executePrecisionSequence();
        }
    }

    /**
     * Execute the precision autonomous sequence using AutoHelper's fluent API
     */
    private void executePrecisionSequence() {
        autoHelper
            // Initial calibration wait
            .waitFor(1000, "System calibration")

            // Move to game piece pickup position
            .moveTo(PICKUP_X, PICKUP_Y, PICKUP_HEADING, "Move to pickup position")

            // Precise alignment for pickup
            .moveTo(PICKUP_X, PICKUP_Y - 2, PICKUP_HEADING, "Precise pickup alignment")

            // Wait for pickup actions
            .waitFor(1500, "Pick up game piece")

            // Move to scoring position
            .moveTo(SCORING_X, SCORING_Y, SCORING_HEADING, "Move to scoring position")

            // Precise scoring alignment
            .turnTo(SCORING_HEADING, "Scoring alignment")

            // Wait for scoring to complete
            .waitFor(2000, "Score game piece")

            // Move to park position
            .moveTo(PARK_X, PARK_Y, PARK_HEADING, "Move to park")

            // Final wait
            .waitFor(1000, "Autonomous complete")

            // Execute all steps
            .executeAll();

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();
    }
    
    /**
     * Alternative step-based execution (for reference - fluent API above is recommended)
     * NOTE: Many of these methods don't exist in AutoHelper - use fluent API instead
     */
    private void executeStep(int step) {
        switch (step) {
            case 0:
                // Initial positioning and calibration
                telemetry.addData("Step 0", "System calibrated");
                sleep(1000);
                break;
                
            case 1:
                // Move to game piece pickup position
                telemetry.addData("Step 1", "Moving to pickup position");
                autoHelper.moveTo(PICKUP_X, PICKUP_Y, PICKUP_HEADING, "Move to pickup").executeAll();
                break;
                
            case 2:
                // Precise alignment for pickup
                telemetry.addData("Step 2", "Precise pickup alignment");
                autoHelper.moveTo(PICKUP_X, PICKUP_Y - 2, PICKUP_HEADING, "Precise alignment").executeAll();
                break;
                
            case 3:
                // Move to scoring position
                telemetry.addData("Step 3", "Moving to scoring position");
                autoHelper.moveTo(SCORING_X, SCORING_Y, SCORING_HEADING, "Move to scoring").executeAll();
                break;
                
            case 4:
                // Precise scoring alignment
                telemetry.addData("Step 4", "Scoring alignment");
                autoHelper.turnTo(SCORING_HEADING, "Scoring alignment").executeAll();
                break;
                
            case 5:
                // Wait for scoring to complete
                telemetry.addData("Step 5", "Scoring complete");
                sleep(2000);
                break;
                
            case 6:
                // Move to park position
                telemetry.addData("Step 6", "Moving to park");
                autoHelper.moveTo(PARK_X, PARK_Y, PARK_HEADING, "Move to park").executeAll();
                break;
                
            case 7:
                // Final status report
                telemetry.addData("Status", "Autonomous Complete!");
                sleep(1000);
                break;
                
            default:
                // End of sequence
                telemetry.addData("Status", "All steps complete");
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
        sleep(1000); // Use LinearOpMode's sleep method
        return true;
    }
    
    /**
     * Example: Score game piece
     */
    private boolean scoreGamePiece() {
        // TODO: Implement your scoring mechanism
        // Example: scoringServo.setPosition(SCORE_POSITION);
        sleep(1500); // Use LinearOpMode's sleep method
        return true;
    }
    
    /**
     * Example: Check if AprilTag correction is needed
     */
    private void checkPositionCorrection() {
        // Access positioning helper through AutoHelper if needed
        telemetry.addData("Position Correction", "Check AprilTag status in telemetry");
    }
}
