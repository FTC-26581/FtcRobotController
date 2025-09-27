package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

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
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1"); // Initialize with camera support

        // TODO: Add any additional hardware initialization here if needed
        
        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        // ========================================
        // ADD YOUR AUTONOMOUS COMMANDS HERE
        // ========================================
        
        // Example autonomous sequence using AutoHelper's fluent API:
        autoHelper
            // Drive forward 24 inches
            .forward(24, "Drive forward 24 inches")

            // Turn right 90 degrees
            .turnBy(90, "Turn right 90 degrees")

            // Strafe left 12 inches
            .strafeLeft(12, "Strafe left 12 inches")

            // Wait for 1 second
            .waitFor(1000, "Wait 1 second")

            // Execute all movements
            .executeAll();

        // Your autonomous code goes here - replace the example above
        // Available AutoHelper fluent API functions:
        // - .forward(inches, "description")
        // - .back(inches, "description")
        // - .strafeLeft(inches, "description")
        // - .strafeRight(inches, "description")
        // - .turnBy(degrees, "description")  // positive = right, negative = left
        // - .turnTo(heading, "description")  // absolute heading
        // - .moveTo(x, y, heading, "description")  // field coordinates
        // - .waitFor(milliseconds, "description")
        // - .executeAll()  // Execute all queued movements

        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();
    }

}