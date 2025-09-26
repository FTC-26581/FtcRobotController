package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AdvancedPositioningHelper;
import org.firstinspires.ftc.teamcode.util.DeadWheelOdometry;

/**
 * DeadWheelTuning - Calibration and Tuning for Dead Wheel Odometry
 * 
 * This OpMode provides tools to calibrate and tune your dead wheel odometry system
 * for maximum accuracy. It includes:
 * 
 * 1. Track Width Tuning - Calibrate the distance between left and right dead wheels
 * 2. Horizontal Offset Tuning - Calibrate the distance from center of rotation to horizontal wheel
 * 3. Encoder Direction Testing - Verify encoder directions are correct
 * 4. Real-time Position Monitoring - Watch position updates in real-time
 * 
 * Instructions:
 * 1. Place robot at known starting position
 * 2. Use d-pad to select tuning mode
 * 3. Follow on-screen instructions for each test
 * 4. Record the calibrated values for use in your robot constants
 * 
 * Controls:
 * - D-pad Up/Down: Select tuning mode
 * - A: Start/Execute current test
 * - B: Reset position to (0,0,0)
 * - X: Toggle debug mode
 * - Y: Save current values (display only)
 * 
 * @author FTC Team
 * @version 1.0
 */
@TeleOp(name="Dead Wheel Tuning", group="Tuning")
public class DeadWheelTuning extends LinearOpMode {
    
    // Tuning system
    private DeadWheelOdometry deadWheelOdometry;
    private AdvancedPositioningHelper positioningHelper;
    
    // Tuning modes
    private enum TuningMode {
        TRACK_WIDTH("Track Width Tuning"),
        HORIZONTAL_OFFSET("Horizontal Offset Tuning"), 
        ENCODER_TEST("Encoder Direction Test"),
        POSITION_MONITOR("Position Monitoring"),
        ACCURACY_TEST("Accuracy Test");
        
        public final String description;
        TuningMode(String description) { this.description = description; }
    }
    
    private TuningMode currentMode = TuningMode.POSITION_MONITOR;
    private boolean debugMode = false;
    
    // Tuning parameters (starting values - will be adjusted)
    private double trackWidth = 15.0;          // Distance between left/right wheels
    private double horizontalOffset = 6.0;     // Distance from center to horizontal wheel
    
    // Test parameters
    private static final double TRACK_WIDTH_TEST_ROTATIONS = 10.0;  // Number of 360° rotations
    private static final double HORIZONTAL_TEST_DISTANCE = 48.0;    // Distance to strafe (inches)
    
    @Override
    public void runOpMode() {
        telemetry.addData("Dead Wheel Tuning", "Initializing...");
        telemetry.update();
        
        // Initialize positioning system with current parameters
        positioningHelper = new AdvancedPositioningHelper(this);
        
        try {
            positioningHelper.initialize(null, true, trackWidth, horizontalOffset);
            deadWheelOdometry = positioningHelper.getDeadWheelOdometry();
            
            telemetry.addData("Initialization", "Complete");
            telemetry.addData("Dead Wheel Status", positioningHelper.getOdometryStatus());
            
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize dead wheels");
            telemetry.addData("Message", e.getMessage());
            telemetry.addData("Check", "Hardware connections and config");
            telemetry.update();
            
            waitForStart();
            return; // Exit if initialization failed
        }
        
        telemetry.addData("", "");
        telemetry.addData("Controls:", "");
        telemetry.addData("D-pad Up/Down", "Select mode");
        telemetry.addData("A", "Execute test");
        telemetry.addData("B", "Reset position");
        telemetry.addData("X", "Toggle debug");
        telemetry.addData("Y", "Show current values");
        telemetry.addData("", "");
        telemetry.addData("Ready", "Press START");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleInput();
            updateTelemetry();
            sleep(50);
        }
    }
    
    /**
     * Handle gamepad input for mode selection and commands
     */
    private void handleInput() {
        // Mode selection
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            currentMode = TuningMode.values()[(currentMode.ordinal() - 1 + TuningMode.values().length) % TuningMode.values().length];
            sleep(200); // Debounce
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            currentMode = TuningMode.values()[(currentMode.ordinal() + 1) % TuningMode.values().length];
            sleep(200); // Debounce
        }
        
        // Command buttons
        if (gamepad1.a) {
            executeCurrentTest();
            sleep(300); // Debounce
        }
        
        if (gamepad1.b) {
            positioningHelper.resetPosition(0, 0, 0);
            sleep(200); // Debounce
        }
        
        if (gamepad1.x) {
            debugMode = !debugMode;
            sleep(200); // Debounce
        }
        
        if (gamepad1.y) {
            showCurrentValues();
            sleep(200); // Debounce
        }
    }
    
    /**
     * Execute the test for the current tuning mode
     */
    private void executeCurrentTest() {
        switch (currentMode) {
            case TRACK_WIDTH:
                performTrackWidthTest();
                break;
            case HORIZONTAL_OFFSET:
                performHorizontalOffsetTest();
                break;
            case ENCODER_TEST:
                performEncoderDirectionTest();
                break;
            case ACCURACY_TEST:
                performAccuracyTest();
                break;
            case POSITION_MONITOR:
                // No specific test - just monitoring
                break;
        }
    }
    
    /**
     * Test track width by rotating robot and measuring encoder difference
     */
    private void performTrackWidthTest() {
        telemetry.addData("Track Width Test", "Starting...");
        telemetry.addData("Instructions", "Robot will rotate %d times", (int)TRACK_WIDTH_TEST_ROTATIONS);
        telemetry.addData("", "Press A again to start");
        telemetry.update();
        
        if (!gamepad1.a) return; // Wait for confirmation
        
        // Reset position and encoders
        positioningHelper.resetPosition(0, 0, 0);
        int[] startEncoders = deadWheelOdometry.getRawEncoderValues();
        
        telemetry.addData("Test Running", "Rotate robot %d full turns clockwise", (int)TRACK_WIDTH_TEST_ROTATIONS);
        telemetry.addData("Current Heading", "%.1f°", positioningHelper.getCurrentHeading());
        telemetry.update();
        
        // Wait for rotation to complete
        double targetHeading = TRACK_WIDTH_TEST_ROTATIONS * 360.0;
        ElapsedTime timeout = new ElapsedTime();
        
        while (opModeIsActive() && Math.abs(positioningHelper.getCurrentHeading() - targetHeading) > 5.0 && timeout.seconds() < 30.0) {
            positioningHelper.updatePosition();
            telemetry.addData("Target Heading", "%.1f°", targetHeading);
            telemetry.addData("Current Heading", "%.1f°", positioningHelper.getCurrentHeading());
            telemetry.addData("Remaining", "%.1f°", targetHeading - positioningHelper.getCurrentHeading());
            telemetry.update();
            sleep(100);
        }
        
        // Calculate new track width
        int[] endEncoders = deadWheelOdometry.getRawEncoderValues();
        int leftDelta = endEncoders[0] - startEncoders[0];
        int rightDelta = endEncoders[1] - startEncoders[1];
        
        double actualRotation = Math.toRadians(positioningHelper.getCurrentHeading());
        double encoderDifference = Math.abs(leftDelta - rightDelta);
        double measuredTrackWidth = encoderDifference / (deadWheelOdometry.getCountsPerInch() * actualRotation);
        
        telemetry.addData("=== TRACK WIDTH RESULTS ===", "");
        telemetry.addData("Encoder Deltas", "L:%d R:%d", leftDelta, rightDelta);
        telemetry.addData("Current Track Width", "%.3f inches", trackWidth);
        telemetry.addData("Measured Track Width", "%.3f inches", measuredTrackWidth);
        telemetry.addData("Difference", "%.3f inches", measuredTrackWidth - trackWidth);
        telemetry.addData("", "");
        telemetry.addData("Recommended", "Use %.3f inches", measuredTrackWidth);
        telemetry.update();
        
        // Update track width for future tests
        trackWidth = measuredTrackWidth;
        
        sleep(5000); // Show results
    }
    
    /**
     * Test horizontal offset by strafing and measuring position drift
     */
    private void performHorizontalOffsetTest() {
        telemetry.addData("Horizontal Offset Test", "Starting...");
        telemetry.addData("Instructions", "Robot will strafe left %.0f inches", HORIZONTAL_TEST_DISTANCE);
        telemetry.addData("", "Press A again to start");
        telemetry.update();
        
        if (!gamepad1.a) return; // Wait for confirmation
        
        // Reset position
        positioningHelper.resetPosition(0, 0, 0);
        double startHeading = positioningHelper.getCurrentHeading();
        
        telemetry.addData("Test Running", "Strafe robot left %.0f inches", HORIZONTAL_TEST_DISTANCE);
        telemetry.addData("Keep heading", "%.1f° (current: %.1f°)", startHeading, positioningHelper.getCurrentHeading());
        telemetry.update();
        
        // Wait for strafe to complete
        ElapsedTime timeout = new ElapsedTime();
        
        while (opModeIsActive() && Math.abs(positioningHelper.getCurrentX()) < HORIZONTAL_TEST_DISTANCE - 2.0 && timeout.seconds() < 20.0) {
            positioningHelper.updatePosition();
            telemetry.addData("Target Position", "X:%.1f Y:0.0", -HORIZONTAL_TEST_DISTANCE);
            telemetry.addData("Current Position", "X:%.1f Y:%.1f", positioningHelper.getCurrentX(), positioningHelper.getCurrentY());
            telemetry.addData("Heading Drift", "%.1f°", positioningHelper.getCurrentHeading() - startHeading);
            telemetry.update();
            sleep(100);
        }
        
        // Analyze results
        double finalX = positioningHelper.getCurrentX();
        double finalY = positioningHelper.getCurrentY();
        double headingDrift = positioningHelper.getCurrentHeading() - startHeading;
        
        // Calculate corrected horizontal offset
        double yDriftPerInchStrafe = finalY / Math.abs(finalX);
        double correctedOffset = horizontalOffset - (yDriftPerInchStrafe * trackWidth / 2.0);
        
        telemetry.addData("=== HORIZONTAL OFFSET RESULTS ===", "");
        telemetry.addData("Target Movement", "X:%.1f Y:0.0", -HORIZONTAL_TEST_DISTANCE);
        telemetry.addData("Actual Movement", "X:%.1f Y:%.1f", finalX, finalY);
        telemetry.addData("Y Drift", "%.3f inches", finalY);
        telemetry.addData("Heading Drift", "%.1f°", headingDrift);
        telemetry.addData("", "");
        telemetry.addData("Current Offset", "%.3f inches", horizontalOffset);
        telemetry.addData("Recommended Offset", "%.3f inches", correctedOffset);
        telemetry.update();
        
        // Update horizontal offset for future tests
        horizontalOffset = correctedOffset;
        
        sleep(5000); // Show results
    }
    
    /**
     * Test encoder directions by moving robot and checking encoder signs
     */
    private void performEncoderDirectionTest() {
        telemetry.addData("Encoder Direction Test", "Starting...");
        telemetry.addData("Instructions", "Move robot forward slowly");
        telemetry.addData("", "Watch encoder values");
        telemetry.update();
        
        int[] startEncoders = deadWheelOdometry.getRawEncoderValues();
        ElapsedTime testTime = new ElapsedTime();
        
        while (opModeIsActive() && testTime.seconds() < 10.0) {
            positioningHelper.updatePosition();
            int[] currentEncoders = deadWheelOdometry.getRawEncoderValues();
            
            telemetry.addData("=== ENCODER DIRECTION TEST ===", "");
            telemetry.addData("Time Remaining", "%.1f seconds", 10.0 - testTime.seconds());
            telemetry.addData("", "");
            telemetry.addData("Left Encoder", "%d (Δ%d)", currentEncoders[0], currentEncoders[0] - startEncoders[0]);
            telemetry.addData("Right Encoder", "%d (Δ%d)", currentEncoders[1], currentEncoders[1] - startEncoders[1]);
            telemetry.addData("Horizontal Encoder", "%d (Δ%d)", currentEncoders[2], currentEncoders[2] - startEncoders[2]);
            telemetry.addData("", "");
            telemetry.addData("Expected for Forward:", "");
            telemetry.addData("Left & Right", "POSITIVE increase");
            telemetry.addData("Horizontal", "Should be ~0");
            telemetry.addData("", "");
            telemetry.addData("Move robot forward", "to test encoder directions");
            telemetry.update();
            
            sleep(100);
        }
        
        sleep(2000);
    }
    
    /**
     * Test overall accuracy with a known movement pattern
     */
    private void performAccuracyTest() {
        telemetry.addData("Accuracy Test", "Starting...");
        telemetry.addData("Instructions", "Robot will move in a square");
        telemetry.addData("", "Press A again to start");
        telemetry.update();
        
        if (!gamepad1.a) return; // Wait for confirmation
        
        double[] startPosition = positioningHelper.getCurrentPose();
        
        // Move in small square pattern (DECODE field coordinates)
        double[][] waypoints = {
            {12, 0, 0},     // Toward Audience 12"
            {12, 12, 90},   // Toward Blue Alliance 12"
            {0, 12, 180},   // Away from Audience 12"
            {0, 0, 270},    // Toward Red Alliance 12"
            {0, 0, 0}       // Return to start
        };
        
        for (int i = 0; i < waypoints.length && opModeIsActive(); i++) {
            while (opModeIsActive() && !positioningHelper.goToPosition(waypoints[i][0], waypoints[i][1], waypoints[i][2], 0.3)) {
                positioningHelper.updatePosition();
                positioningHelper.updateTelemetry();
                telemetry.addData("Waypoint", "%d/%d", i+1, waypoints.length);
                telemetry.update();
                sleep(50);
            }
            sleep(500);
        }
        
        // Calculate final accuracy
        double[] finalPosition = positioningHelper.getCurrentPose();
        double positionError = Math.sqrt(
            Math.pow(finalPosition[0] - startPosition[0], 2) + 
            Math.pow(finalPosition[1] - startPosition[1], 2)
        );
        double headingError = Math.abs(Math.toDegrees(finalPosition[2]) - Math.toDegrees(startPosition[2]));
        
        telemetry.addData("=== ACCURACY TEST RESULTS ===", "");
        telemetry.addData("Start Position", "X:%.2f Y:%.2f H:%.1f°", 
            startPosition[0], startPosition[1], Math.toDegrees(startPosition[2]));
        telemetry.addData("End Position", "X:%.2f Y:%.2f H:%.1f°", 
            finalPosition[0], finalPosition[1], Math.toDegrees(finalPosition[2]));
        telemetry.addData("Position Error", "%.3f inches", positionError);
        telemetry.addData("Heading Error", "%.1f degrees", headingError);
        telemetry.addData("", "");
        telemetry.addData("Status", positionError < 0.5 ? "EXCELLENT" : 
                         positionError < 1.5 ? "GOOD" : "NEEDS TUNING");
        telemetry.update();
        
        sleep(5000);
    }
    
    /**
     * Show current calibrated values
     */
    private void showCurrentValues() {
        telemetry.addData("=== CURRENT VALUES ===", "");
        telemetry.addData("Track Width", "%.3f inches", trackWidth);
        telemetry.addData("Horizontal Offset", "%.3f inches", horizontalOffset);
        telemetry.addData("", "");
        telemetry.addData("Java Constants:", "");
        telemetry.addData("TRACK_WIDTH", "%.3f", trackWidth);
        telemetry.addData("HORIZONTAL_OFFSET", "%.3f", horizontalOffset);
        telemetry.addData("", "");
        telemetry.addData("Copy these values", "to your robot constants");
        telemetry.update();
        sleep(3000);
    }
    
    /**
     * Update telemetry based on current mode
     */
    private void updateTelemetry() {
        // Update position
        positioningHelper.updatePosition();
        
        // Mode header
        telemetry.addData("=== " + currentMode.description.toUpperCase() + " ===", "");
        
        switch (currentMode) {
            case POSITION_MONITOR:
                showPositionMonitoring();
                break;
            case TRACK_WIDTH:
                telemetry.addData("Current Track Width", "%.3f inches", trackWidth);
                telemetry.addData("Test", "Rotate robot %d times", (int)TRACK_WIDTH_TEST_ROTATIONS);
                telemetry.addData("Press A", "to start test");
                break;
            case HORIZONTAL_OFFSET:
                telemetry.addData("Current Offset", "%.3f inches", horizontalOffset);
                telemetry.addData("Test", "Strafe %.0f inches", HORIZONTAL_TEST_DISTANCE);
                telemetry.addData("Press A", "to start test");
                break;
            case ENCODER_TEST:
                telemetry.addData("Test", "Check encoder directions");
                telemetry.addData("Press A", "to start 10-second test");
                break;
            case ACCURACY_TEST:
                telemetry.addData("Test", "Square movement pattern");
                telemetry.addData("Press A", "to start test");
                break;
        }
        
        telemetry.addData("", "");
        
        // Current position
        telemetry.addData("Position", "X:%.2f Y:%.2f H:%.1f°", 
            positioningHelper.getCurrentX(), 
            positioningHelper.getCurrentY(), 
            positioningHelper.getCurrentHeading());
        
        // Debug information
        if (debugMode) {
            int[] encoders = deadWheelOdometry.getRawEncoderValues();
            telemetry.addData("Raw Encoders", "L:%d R:%d H:%d", encoders[0], encoders[1], encoders[2]);
            telemetry.addData("System Status", positioningHelper.getOdometryStatus());
        }
        
        // Controls
        telemetry.addData("", "");
        telemetry.addData("D-pad Up/Down", "Change mode");
        telemetry.addData("A", "Execute test");
        telemetry.addData("B", "Reset position");
        telemetry.addData("X", "Debug: " + (debugMode ? "ON" : "OFF"));
        telemetry.addData("Y", "Show values");
        
        telemetry.update();
    }
    
    /**
     * Show detailed position monitoring information
     */
    private void showPositionMonitoring() {
        telemetry.addData("Live Position", "X:%.3f Y:%.3f H:%.2f°",
            positioningHelper.getCurrentX(),
            positioningHelper.getCurrentY(), 
            positioningHelper.getCurrentHeading());
        
        if (deadWheelOdometry != null) {
            int[] encoders = deadWheelOdometry.getRawEncoderValues();
            telemetry.addData("Encoders", "L:%d R:%d H:%d", encoders[0], encoders[1], encoders[2]);
            
            double[] velocity = deadWheelOdometry.getVelocity();
            telemetry.addData("Velocity", "X:%.2f Y:%.2f ω:%.1f°/s", 
                velocity[0], velocity[1], Math.toDegrees(velocity[2]));
        }
        
        telemetry.addData("Tuning Parameters", "");
        telemetry.addData("Track Width", "%.3f inches", trackWidth);
        telemetry.addData("Horizontal Offset", "%.3f inches", horizontalOffset);
    }
}