package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Simple Autonomous Template - Ready to Use
 * 
 * This template uses AutoHelper for super simple autonomous programming.
 * Just add your movement commands in the main section and you're ready to go!
 * 
 * To customize:
 * 1. Uncomment @Autonomous line below
 * 2. Change the name and group as desired
 * 3. Add your movement commands in the main autonomous section
 * 4. Adjust speeds as needed
 */

// @Autonomous(name = "Simple Auto", group = "Auto")
public class SimpleAutoTemplate extends LinearOpMode {
    
    // AutoHelper handles all the complex stuff for us
    private AutoHelper autoHelper;
    
    // Movement speeds - adjust these as needed
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {
        
        // Initialize AutoHelper - it handles all hardware setup automatically
        autoHelper = new AutoHelper(this);
        autoHelper.initialize(); // Uses basic mechanum drive by default
        
        // TODO: Add any additional hardware initialization here if needed
        
        autoHelper.addTelemetry("Status", "Initialized - Ready to Start");
        autoHelper.updateTelemetry();
        
        waitForStart();
        autoHelper.resetRuntime();
        
        // ========================================
        // ADD YOUR AUTONOMOUS COMMANDS HERE
        // ========================================
        
        // Example autonomous sequence (uncomment to use):
        /*
        // Drive forward for 2 seconds
        autoHelper.driveForward(2.0, DRIVE_SPEED);
        
        // Turn right for 1 second
        autoHelper.turnRight(1.0, TURN_SPEED);
        
        // Strafe left for 1.5 seconds
        autoHelper.strafeLeft(1.5, DRIVE_SPEED);
        
        // Wait for 1 second
        autoHelper.waitFor(1.0);
        */
        
        // Your autonomous code goes here - replace the example above
        // Available functions:
        // - autoHelper.driveForward(seconds, speed)
        // - autoHelper.driveBackward(seconds, speed)
        // - autoHelper.strafeLeft(seconds, speed)
        // - autoHelper.strafeRight(seconds, speed)
        // - autoHelper.turnLeft(seconds, speed)
        // - autoHelper.turnRight(seconds, speed)
        // - autoHelper.waitFor(seconds)
        // - autoHelper.driveFor(seconds, forward, strafe, rotate)
        //
        // For encoder-based movement:
        // - autoHelper.driveForwardDistance(inches, speed)
        // - autoHelper.driveBackwardDistance(inches, speed)
        // - autoHelper.strafeLeftDistance(inches, speed)
        // - autoHelper.strafeRightDistance(inches, speed)
        // - autoHelper.turnLeftDegrees(degrees, speed)
        // - autoHelper.turnRightDegrees(degrees, speed)
        
        
        autoHelper.addTelemetry("Status", "Autonomous Complete!");
        autoHelper.updateTelemetry();
    }

}