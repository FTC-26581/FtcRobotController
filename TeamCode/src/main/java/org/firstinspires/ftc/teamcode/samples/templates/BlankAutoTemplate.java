package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        autoHelper = new AutoHelper(this);
        autoHelper.initialize(AutoHelper.DriveMode.BASIC_MECHANUM);
        
        // Option 2: Field Relative Drive (maintains orientation relative to field)
        // autoHelper = new AutoHelper(this);
        // autoHelper.initialize(AutoHelper.DriveMode.FIELD_RELATIVE);
        
        // Option 3: Advanced Positioning (encoder + IMU + AprilTag fusion for maximum precision)
        // autoHelper = new AutoHelper(this);
        // autoHelper.initialize(AutoHelper.DriveMode.ADVANCED_POSITIONING, "Webcam 1");
        // autoHelper.setCurrentPosition(0, 0, 0); // Set starting position
        
        // TODO: Add any additional hardware initialization here
        // Example: servos, sensors, cameras, etc.
        
        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();
        
        waitForStart();
        autoHelper.resetRuntime();
        
        // Main autonomous loop
        while (opModeIsActive()) {
            
            // Update position tracking (only needed for advanced positioning)
            autoHelper.updatePosition();
            
            // Execute current step
            executeStep(autoHelper.getCurrentStep());
            
            // Update telemetry
            autoHelper.updateTelemetry();
            
            // Check if autonomous should end
            if (autoHelper.getCurrentStep() >= getTotalSteps()) {
                autoHelper.addTelemetry("Status", "Autonomous Complete!");
                autoHelper.updateTelemetry();
                break;
            }
        }
        
        // Clean up resources
        autoHelper.close();
    }
    

    
    /**
     * Main step execution logic - ADD YOUR AUTONOMOUS STEPS HERE
     */
    private void executeStep(int step) {
        switch (step) {
            case 0:
                // Example: Wait at start
                if (autoHelper.waitFor(1.0)) {
                    autoHelper.addTelemetry("Step 0", "Initial wait complete");
                }
                break;
                
            case 1:
                // Example: Drive forward 24 inches using encoders
                if (autoHelper.driveForwardDistance(24, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 1", "Forward drive complete");
                }
                break;
                
            case 2:
                // Example: Strafe right 12 inches using encoders
                if (autoHelper.strafeRightDistance(12, STRAFE_SPEED)) {
                    autoHelper.addTelemetry("Step 2", "Strafe right complete");
                }
                break;
                
            case 3:
                // Example: Turn 90 degrees using encoders
                if (autoHelper.turnLeftDegrees(90, TURN_SPEED)) {
                    autoHelper.addTelemetry("Step 3", "Turn complete");
                }
                break;
                
            case 4:
                // Example: Timed movement (drive forward for 2 seconds)
                if (autoHelper.driveForward(2.0, 0.5)) {
                    autoHelper.addTelemetry("Step 4", "Timed movement complete");
                }
                break;
                
            case 5:
                // Example: More complex movement - drive diagonally
                if (autoHelper.driveFor(1.5, 0.5, 0.3, 0)) {
                    autoHelper.addTelemetry("Step 5", "Diagonal movement complete");
                }
                break;
                
            case 6:
                // Example: Advanced positioning - go to exact field coordinates
                // (Only works if initialized with ADVANCED_POSITIONING mode)
                if (autoHelper.goToPosition(36.0, 24.0, 90.0, DRIVE_SPEED)) {
                    autoHelper.addTelemetry("Step 6", "Precise position reached");
                    autoHelper.addTelemetry("Position", "X:%.1f Y:%.1f H:%.1f°", 
                            autoHelper.getCurrentX(), autoHelper.getCurrentY(), autoHelper.getCurrentHeading());
                }
                break;
                
            case 7:
                // Example: Precise rotation to exact heading
                if (autoHelper.rotateToHeading(180.0, TURN_SPEED)) {
                    autoHelper.addTelemetry("Step 7", "Precise rotation complete");
                    autoHelper.addTelemetry("Heading", "%.1f°", autoHelper.getCurrentHeading());
                }
                break;
                
            // TODO: Add more steps as needed
            // case 8:
            //     if (yourCustomFunction()) {
            //         autoHelper.addTelemetry("Step 8", "Custom action complete");
            //     }
            //     break;
                
            default:
                // End of autonomous - this will exit the main loop
                autoHelper.nextStep();
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
        
        // For now, just wait 1 second as an example
        return autoHelper.waitFor(1.0);
    }
    
    /**
     * Example: Custom function for scoring a game piece
     * @return true when action is complete
     */
    private boolean scoreGamePiece() {
        // TODO: Add your custom code here
        // Example: Move lift, activate servos, etc.
        
        // For now, just wait 1 second as an example
        return autoHelper.waitFor(1.0);
    }
    
    // TODO: Add more custom functions as needed
    // Examples:
    // - alignToTarget()
    // - detectGamePiece()
    // - moveToScoringPosition()
    // - initializeSpecialHardware()
}