package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AdvancedPositioningHelper;
import org.firstinspires.ftc.teamcode.util.DeadWheelOdometry;

/**
 * DeadWheelOdometryDemo - Demonstration of Dead Wheel Odometry System
 * 
 * This OpMode demonstrates how to use the DeadWheelOdometry class for high-accuracy
 * robot positioning. It shows:
 * 1. Basic setup and initialization
 * 2. Position tracking during movement
 * 3. Tuning parameters for your specific robot
 * 4. Integration with AdvancedPositioningHelper
 * 
 * Hardware Requirements:
 * - Three encoder wheels (two parallel, one perpendicular)
 * - Encoders connected to hardware map as "leftOdometer", "rightOdometer", "horizontalOdometer"
 * - IMU for heading reference
 * 
 * Setup Instructions:
 * 1. Mount dead wheels on your robot with proper spacing
 * 2. Measure track width (distance between left and right wheels)
 * 3. Measure horizontal offset (distance from center of rotation to horizontal wheel)
 * 4. Adjust the constants in this demo for your robot
 * 5. Run tuning procedures to optimize accuracy
 * 
 * @author FTC Team
 * @version 1.0
 */
@Autonomous(name="Dead Wheel Odometry Demo", group="Demo")
public class DeadWheelOdometryDemo extends LinearOpMode {
    
    // Dead wheel odometry system
    private DeadWheelOdometry deadWheelOdometry;
    private AdvancedPositioningHelper positioningHelper;
    
    // Demo constants - adjust for your robot
    private static final double TRACK_WIDTH = 15.0;        // Distance between left/right dead wheels (inches)
    private static final double HORIZONTAL_OFFSET = 6.0;   // Distance from center to horizontal wheel (inches)
    
    // Demo movement parameters
    private static final double DEMO_DISTANCE = 24.0;     // Distance to move for demo (inches)
    private static final double DEMO_SPEED = 0.5;         // Movement speed
    
    @Override
    public void runOpMode() {
        telemetry.addData("Dead Wheel Demo", "Initializing...");
        telemetry.update();
        
        // Initialize positioning system with dead wheels
        positioningHelper = new AdvancedPositioningHelper(this);
        
        try {
            // Method 1: Use default parameters (automatically detected)
            // positioningHelper.initialize("Webcam 1", true);
            
            // Method 2: Use custom parameters for your robot
            positioningHelper.initialize("Webcam 1", true, TRACK_WIDTH, HORIZONTAL_OFFSET);
            
            telemetry.addData("Initialization", "Complete");
            telemetry.addData("Dead Wheel Status", positioningHelper.getOdometryStatus());
            telemetry.addData("", "");
            telemetry.addData("Instructions", "This demo will:");
            telemetry.addData("1.", "Show current position tracking");
            telemetry.addData("2.", "Move in a square pattern");
            telemetry.addData("3.", "Display accuracy comparison");
            telemetry.addData("", "");
            telemetry.addData("Ready", "Press START to begin");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize dead wheels");
            telemetry.addData("Message", e.getMessage());
            telemetry.addData("Fallback", "Using motor encoders instead");
            telemetry.update();
            
            // Fall back to motor encoders
            positioningHelper.initialize("Webcam 1", false);
        }
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Reset position to field center
            positioningHelper.resetPosition(0, 0, 0);
            
            // Demo 1: Position tracking while stationary
            demonstratePositionTracking();
            
            // Demo 2: Square movement pattern
            if (positioningHelper.isUsingDeadWheels()) {
                demonstrateSquareMovement();
            } else {
                telemetry.addData("Note", "Square demo skipped - using motor encoders");
                telemetry.update();
                sleep(2000);
            }
            
            // Demo 3: Accuracy comparison
            demonstrateAccuracyComparison();
            
            telemetry.addData("Demo", "Complete!");
            telemetry.update();
        }
    }
    
    /**
     * Demonstrate position tracking while robot is stationary
     * This shows the stability and precision of the dead wheel system
     */
    private void demonstratePositionTracking() {
        telemetry.addData("Demo 1", "Position Tracking (10 seconds)");
        telemetry.addData("Status", "Keep robot stationary");
        telemetry.update();
        
        ElapsedTime timer = new ElapsedTime();
        double[] initialPosition = positioningHelper.getCurrentPose();
        double maxDrift = 0.0;
        
        while (opModeIsActive() && timer.seconds() < 10.0) {
            positioningHelper.updatePosition();
            
            double[] currentPosition = positioningHelper.getCurrentPose();
            double drift = Math.sqrt(
                Math.pow(currentPosition[0] - initialPosition[0], 2) + 
                Math.pow(currentPosition[1] - initialPosition[1], 2)
            );
            maxDrift = Math.max(maxDrift, drift);
            
            telemetry.addData("=== POSITION TRACKING ===", "");
            telemetry.addData("Time", "%.1f / 10.0 seconds", timer.seconds());
            telemetry.addData("Position", "X:%.3f Y:%.3f H:%.1f°", 
                currentPosition[0], currentPosition[1], Math.toDegrees(currentPosition[2]));
            telemetry.addData("Drift", "Current:%.3f\" Max:%.3f\"", drift, maxDrift);
            telemetry.addData("System", positioningHelper.getOdometryStatus());
            
            if (positioningHelper.isUsingDeadWheels()) {
                DeadWheelOdometry deadWheels = positioningHelper.getDeadWheelOdometry();
                int[] encoderValues = deadWheels.getRawEncoderValues();
                telemetry.addData("Encoders", "L:%d R:%d H:%d", 
                    encoderValues[0], encoderValues[1], encoderValues[2]);
            }
            
            telemetry.update();
            sleep(100);
        }
        
        telemetry.addData("Demo 1 Results", "");
        telemetry.addData("Maximum Drift", "%.3f inches", maxDrift);
        telemetry.addData("Status", maxDrift < 0.1 ? "EXCELLENT" : 
                         maxDrift < 0.5 ? "GOOD" : "NEEDS TUNING");
        telemetry.update();
        sleep(3000);
    }
    
    /**
     * Demonstrate movement in a square pattern
     * This shows accuracy during motion and return-to-start precision
     */
    private void demonstrateSquareMovement() {
        telemetry.addData("Demo 2", "Square Movement Pattern");
        telemetry.addData("Status", "Moving in 24\" square");
        telemetry.update();
        
        double[] startPosition = positioningHelper.getCurrentPose();
        
        // Move in a square: Toward Blue Alliance -> Toward Audience -> Toward Red Alliance -> Away from Audience -> Return to start
        double[][] waypoints = {
            {0, DEMO_DISTANCE, 0},      // Toward Blue Alliance 24"
            {DEMO_DISTANCE, DEMO_DISTANCE, 90},   // Toward Audience 24", turn 90°
            {DEMO_DISTANCE, 0, 180},    // Toward Red Alliance 24", turn 180°
            {0, 0, 270},                // Away from Audience 24", turn 270°
            {0, 0, 0}                   // Return to start heading
        };
        
        for (int i = 0; i < waypoints.length && opModeIsActive(); i++) {
            telemetry.addData("Square Movement", "Waypoint %d/%d", i+1, waypoints.length);
            telemetry.addData("Target", "X:%.1f Y:%.1f H:%.0f°", 
                waypoints[i][0], waypoints[i][1], waypoints[i][2]);
            telemetry.update();
            
            // Move to waypoint
            while (opModeIsActive() && !positioningHelper.goToPosition(
                    waypoints[i][0], waypoints[i][1], waypoints[i][2], DEMO_SPEED)) {
                
                positioningHelper.updateTelemetry();
                telemetry.addData("Target", "X:%.1f Y:%.1f H:%.0f°", 
                    waypoints[i][0], waypoints[i][1], waypoints[i][2]);
                telemetry.addData("Distance", "%.2f inches", 
                    positioningHelper.getDistanceToTarget(waypoints[i][0], waypoints[i][1]));
                telemetry.update();
                
                sleep(50);
            }
            
            // Brief pause at each waypoint
            sleep(500);
        }
        
        // Calculate final position error
        double[] finalPosition = positioningHelper.getCurrentPose();
        double positionError = Math.sqrt(
            Math.pow(finalPosition[0] - startPosition[0], 2) + 
            Math.pow(finalPosition[1] - startPosition[1], 2)
        );
        double headingError = Math.abs(AdvancedPositioningHelper.normalizeAngle(
            Math.toDegrees(finalPosition[2]) - Math.toDegrees(startPosition[2])));
        
        telemetry.addData("Demo 2 Results", "");
        telemetry.addData("Position Error", "%.3f inches", positionError);
        telemetry.addData("Heading Error", "%.1f degrees", headingError);
        telemetry.addData("Status", positionError < 1.0 ? "EXCELLENT" : 
                         positionError < 3.0 ? "GOOD" : "NEEDS TUNING");
        telemetry.update();
        sleep(3000);
    }
    
    /**
     * Demonstrate accuracy comparison between different odometry systems
     */
    private void demonstrateAccuracyComparison() {
        telemetry.addData("Demo 3", "System Comparison");
        telemetry.addData("Status", "Analyzing odometry performance");
        telemetry.update();
        
        if (positioningHelper.isUsingDeadWheels()) {
            DeadWheelOdometry deadWheels = positioningHelper.getDeadWheelOdometry();
            
            telemetry.addData("=== DEAD WHEEL ODOMETRY ===", "");
            telemetry.addData("Track Width", "%.2f inches", deadWheels.getTrackWidth());
            telemetry.addData("Horizontal Offset", "%.2f inches", deadWheels.getHorizontalOffset());
            telemetry.addData("Encoder Resolution", "%.1f counts/inch", deadWheels.getCountsPerInch());
            telemetry.addData("Status", deadWheels.getStatus());
            
            // Get current encoder readings
            int[] encoders = deadWheels.getRawEncoderValues();
            telemetry.addData("Raw Encoders", "L:%d R:%d H:%d", encoders[0], encoders[1], encoders[2]);
            
            // Calculate theoretical vs actual resolution
            double theoreticalResolution = deadWheels.getCountsPerInch();
            telemetry.addData("Theoretical CPR", "%.1f counts/inch", theoreticalResolution);
            
        } else {
            telemetry.addData("=== MOTOR ENCODER ODOMETRY ===", "");
            telemetry.addData("Resolution", "%.1f counts/inch", AdvancedPositioningHelper.COUNTS_PER_INCH);
            telemetry.addData("Wheel Diameter", "%.1f inches", AdvancedPositioningHelper.WHEEL_DIAMETER_INCHES);
            telemetry.addData("Note", "Dead wheels provide higher accuracy");
        }
        
        telemetry.addData("", "");
        telemetry.addData("Advantages of Dead Wheels:", "");
        telemetry.addData("• No wheel slip errors", "");
        telemetry.addData("• Higher encoder resolution", "");
        telemetry.addData("• Dedicated positioning sensors", "");
        telemetry.addData("• Better curved path tracking", "");
        
        telemetry.update();
        sleep(5000);
    }
    
    /**
     * Utility method to format position for telemetry
     */
    private String formatPosition(double[] position) {
        return String.format("X:%.2f Y:%.2f H:%.1f°", 
            position[0], position[1], Math.toDegrees(position[2]));
    }
}
