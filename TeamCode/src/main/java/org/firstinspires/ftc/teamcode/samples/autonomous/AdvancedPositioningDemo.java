package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * Advanced Positioning Demo - Precision Movement Example
 * 
 * This OpMode demonstrates the advanced positioning system that combines:
 * - Encoder odometry for dead reckoning
 * - IMU gyroscope for heading accuracy
 * - AprilTag vision for absolute position correction
 * 
 * The robot will move to precise field coordinates and orientations,
 * showcasing the accuracy of the sensor fusion system using AutoHelper's fluent API.
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
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1");

        telemetry.addData("Status", "Advanced Positioning Initialized");
        telemetry.addData("Starting Position", "X:%.1f Y:%.1f H:%.1f°", START_X, START_Y, START_HEADING);
        telemetry.addData("", "Place robot at starting position and press play");
        telemetry.update();
        
        waitForStart();

        // Execute the autonomous sequence using AutoHelper's fluent API
        autoHelper
            // Wait for initial calibration
            .waitFor(2000, "Initial calibration wait")

            // Move to Position 1 with specific heading
            .moveTo(POSITION_1_X, POSITION_1_Y, POSITION_1_HEADING, "Move to Position 1")

            // Wait to show position accuracy
            .waitFor(2000, "Position accuracy check at Position 1")

            // Move to Position 2
            .moveTo(POSITION_2_X, POSITION_2_Y, POSITION_2_HEADING, "Move to Position 2")

            // Demonstrate pure rotation (180° turn)
            .turnTo(POSITION_2_HEADING + 180, "Rotate 180 degrees")

            // Move to Position 3 (no heading change specified, will maintain current heading)
            .moveTo(POSITION_3_X, POSITION_3_Y, POSITION_3_HEADING, "Move to Position 3")

            // Return to start position with original heading
            .moveTo(START_X, START_Y, START_HEADING, "Return to start position")

            // Final wait to assess accuracy
            .waitFor(3000, "Final accuracy assessment")

            // Execute all the queued steps
            .executeAll();

        telemetry.addData("Status", "Demo Complete!");
        telemetry.update();
    }
}
