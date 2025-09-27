package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * Blank Autonomous Template with Step-Based Control
 * 
 * This template uses the AutoHelper class for clean, easy autonomous programming.
 * 
 * Features:
 * - Step-based autonomous execution
 * - Easy movement functions (time-based and encoder-based)
 * - Advanced positioning with sensor fusion (encoder + IMU + AprilTag)
 * - Automatic hardware initialization
 * - Built-in telemetry and debugging
 * - Integration with Mechanum drive classes
 * 
 * To use this template:
 * 1. Uncomment the @Autonomous annotation
 * 2. Add your steps in the executeStep() method
 * 3. Update getTotalSteps() with your total number of steps
 * 4. Choose drive mode (basic, field-relative, or advanced positioning)
 * 5. Customize movement parameters as needed
 */

// @Autonomous(name = "Blank Auto Template", group = "Template")
public class BlankAutoTemplate extends LinearOpMode {
    
    // AutoHelper handles all the complex autonomous functionality
    private AutoHelper autoHelper;
    
    // Movement speeds - adjust as needed
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final double STRAFE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        
        // Initialize AutoHelper - Choose your drive mode:
        
        // Option 1: Basic Mechanum Drive (simple, reliable)
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1");

        // Option 2: Field Relative Drive (maintains orientation relative to field)
        // autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        // autoHelper.initialize("Webcam 1");

        // Option 3: Advanced Positioning (encoder + IMU + AprilTag fusion for maximum precision)
        // autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        // autoHelper.initialize("Webcam 1");
        // // Set starting position if needed: autoHelper.getPositioningHelper().resetPosition(0, 0, 0);

        // TODO: Add any additional hardware initialization here
        // Example: servos, sensors, cameras, etc.
        
        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();
        
        waitForStart();

        // Main autonomous loop using AutoHelper's fluent API
        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }

    /**
     * Execute the main autonomous sequence using AutoHelper's fluent API
     */
    private void executeAutonomousSequence() {
        // Use AutoHelper's fluent API for cleaner, more reliable autonomous programming
        autoHelper
            // Step 0: Initial wait
            .waitFor(1000, "Initial wait")

            // Step 1: Drive forward 24 inches
            .forward(24, "Drive forward 24 inches")

            // Step 2: Strafe right 12 inches
            .strafeRight(12, "Strafe right 12 inches")

            // Step 3: Turn 90 degrees left
            .turnBy(-90, "Turn 90 degrees left")

            // Step 4: Move to specific position (if using advanced positioning)
            .moveTo(36.0, 24.0, 90.0, "Move to scoring position")

            // Step 5: Wait for action (simulate scoring)
            .waitFor(2000, "Score game piece")

            // Step 6: Return to start area
            .moveTo(0.0, 0.0, 0.0, "Return to start")

            // Execute all steps
            .executeAll();

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();
    }
    

    
    /**
     * Main step execution logic - ADD YOUR AUTONOMOUS STEPS HERE
     * NOTE: This method is optional - the fluent API in executeAutonomousSequence() is recommended
     */
    private void executeStep(int step) {
        switch (step) {
            case 0:
                // Example: Wait at start - use correct waitFor method (milliseconds, not seconds)
                telemetry.addData("Step 0", "Initial wait...");
                sleep(1000); // Simple wait
                break;
                
            case 1:
                // Example: Drive forward 24 inches - use AutoHelper fluent API
                telemetry.addData("Step 1", "Moving forward 24 inches");
                autoHelper.forward(24, "Drive forward 24 inches").executeAll();
                break;
                
            case 2:
                // Example: Strafe right 12 inches - use AutoHelper fluent API
                telemetry.addData("Step 2", "Strafing right 12 inches");
                autoHelper.strafeRight(12, "Strafe right 12 inches").executeAll();
                break;
                
            case 3:
                // Example: Turn 90 degrees - use AutoHelper fluent API
                telemetry.addData("Step 3", "Turning 90 degrees left");
                autoHelper.turnBy(-90, "Turn 90 degrees left").executeAll();
                break;
                
            case 4:
                // Example: Timed movement - use direct motor control or simple movement
                telemetry.addData("Step 4", "Timed movement forward");
                autoHelper.forward(12, "Timed forward movement").executeAll();
                break;
                
            case 5:
                // Example: Complex movement using multiple steps
                telemetry.addData("Step 5", "Complex diagonal movement");
                autoHelper
                    .forward(6, "Forward component")
                    .strafeRight(6, "Right component")
                    .executeAll();
                break;
                
            case 6:
                // Example: Move to specific position (requires AdvancedPositioningHelper)
                telemetry.addData("Step 6", "Moving to precise position");
                autoHelper.moveTo(36.0, 24.0, 90.0, "Move to scoring position").executeAll();
                break;
                
            case 7:
                // Example: Precise rotation to exact heading
                telemetry.addData("Step 7", "Rotating to 180 degrees");
                autoHelper.turnTo(180.0, "Rotate to 180 degrees").executeAll();
                break;
                
            // TODO: Add more steps as needed
            // case 8:
            //     if (yourCustomFunction()) {
            //         telemetry.addData("Step 8", "Custom action complete");
            //     }
            //     break;
                
            default:
                // End of autonomous - mark complete
                telemetry.addData("Status", "All steps complete");
                break;
        }
    }
    
    /**
     * Get total number of steps (update this when adding steps)
     */
    private int getTotalSteps() {
        return 8; // TODO: Update this number when adding more steps
    }
    
    // ========================================
    // CUSTOM FUNCTIONS
    // Add your own custom autonomous functions here
    // ========================================
    
    /**
     * Example: Custom function for picking up a game piece
     * @return true when action is complete
     */
    private boolean pickUpGamePiece() {
        // TODO: Add your custom code here
        // Example: Move servos, wait for sensors, etc.
        
        // For now, just wait 1 second as an example using correct method
        sleep(1000); // Use LinearOpMode's sleep method
        return true;
    }
    
    /**
     * Example: Custom function for scoring a game piece
     * @return true when action is complete
     */
    private boolean scoreGamePiece() {
        // TODO: Add your custom code here
        // Example: Move lift, activate servos, etc.
        
        // For now, just wait 1 second as an example using correct method
        sleep(1000); // Use LinearOpMode's sleep method
        return true;
    }
    
    // TODO: Add more custom functions as needed
    // Examples:
    // - alignToTarget()
    // - detectGamePiece()
    // - moveToScoringPosition()
    // - initializeSpecialHardware()
}
