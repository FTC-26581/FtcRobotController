package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demo OpMode for Unified Odometry System
 * 
 * This OpMode demonstrates the unified odometry system that supports both:
 * 1. Traditional 3-wheel dead wheel systems
 * 2. goBILDA Pinpoint 2-pod systems
 * 
 * The system automatically detects which hardware is available and configures accordingly.
 * 
 * Hardware Requirements (choose one):
 * 
 * Traditional 3-wheel system:
 * - "leftOdometry" - Left dead wheel encoder
 * - "rightOdometry" - Right dead wheel encoder  
 * - "horizontalOdometry" - Horizontal dead wheel encoder
 * 
 * goBILDA Pinpoint system:
 * - "pinpoint" - Pinpoint Odometry Computer connected via I2C
 * - Two odometry pods connected to Pinpoint (X forward, Y strafe)
 * 
 * Controls:
 * - Gamepad1 Left Stick: Move robot (for testing)
 * - Gamepad1 Right Stick X: Rotate robot (for testing)
 * - Gamepad1 A: Reset position to origin
 * - Gamepad1 B: Set position to DECODE field coordinates
 * - Gamepad1 X: Toggle debug mode
 * - Gamepad1 Y: Recalibrate IMU (Pinpoint only)
 * 
 * @author FTC Team
 */
@TeleOp(name="Unified Odometry Demo", group="Demo")
public class UnifiedOdometryDemo extends LinearOpMode {
    
    // Odometry system
    private DeadWheelOdometry odometry;
    
    // Performance tracking
    private ElapsedTime runtime = new ElapsedTime();
    
    // Demo settings
    private boolean debugMode = false;
    
    @Override
    public void runOpMode() {
        
        // Initialize odometry system (auto-detects hardware)
        odometry = new DeadWheelOdometry(hardwareMap, telemetry);
        
        // Display initialization results
        telemetry.addData("=== UNIFIED ODOMETRY DEMO ===", "");
        telemetry.addData("System Type", odometry.getSystemType());
        telemetry.addData("System Status", odometry.getSystemStatus());
        telemetry.addData("Initialized", odometry.isInitialized());
        
        if (odometry.isInitialized()) {
            telemetry.addData("", "");
            telemetry.addData("Ready to Start", "Press PLAY when robot is positioned");
            
            if (odometry.isPinpointSystem()) {
                telemetry.addData("Pinpoint Info", "2-pod system with internal IMU");
                telemetry.addData("Features", "High accuracy, velocity data, auto-calibration");
            } else if (odometry.isTraditionalSystem()) {
                telemetry.addData("Traditional Info", "3-wheel dead wheel system");
                telemetry.addData("Features", "Reliable, well-tested, manual tuning");
            }
        } else {
            telemetry.addData("ERROR", "Odometry system failed to initialize");
            telemetry.addData("Check", "Hardware connections and configuration");
        }
        
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("A", "Reset position to origin");
        telemetry.addData("B", "Set DECODE field position");
        telemetry.addData("X", "Toggle debug mode");
        telemetry.addData("Y", "Recalibrate IMU (Pinpoint only)");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
        // Reset position at start
        odometry.resetPosition();
        
        while (opModeIsActive()) {
            
            // Update odometry
            odometry.updatePosition();
            
            // Handle controls
            handleControls();
            
            // Display telemetry
            displayTelemetry();
            
            telemetry.update();
        }
    }
    
    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        
        // Reset position to origin
        if (gamepad1.a) {
            odometry.resetPosition(0.0, 0.0, 0.0);
            telemetry.addData("Action", "Position reset to origin");
        }
        
        // Set position to DECODE field coordinates (example position)
        if (gamepad1.b) {
            // Example: Robot starting at Red Alliance side, facing Blue Alliance
            odometry.resetPosition(-48.0, -48.0, 0.0); // DECODE field coordinates
            telemetry.addData("Action", "Position set to DECODE field start");
        }
        
        // Toggle debug mode
        if (gamepad1.x) {
            debugMode = !debugMode;
            telemetry.addData("Action", "Debug mode " + (debugMode ? "ON" : "OFF"));
            try {
                Thread.sleep(200); // Debounce
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        
        // Recalibrate IMU (Pinpoint only)
        if (gamepad1.y) {
            odometry.recalibrateIMU();
            telemetry.addData("Action", "IMU recalibration requested");
        }
    }
    
    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        
        // Runtime
        telemetry.addData("Runtime", String.format("%.1f sec", runtime.seconds()));
        telemetry.addData("", "");
        
        // System information
        telemetry.addData("=== SYSTEM INFO ===", "");
        telemetry.addData("Odometry Type", odometry.getSystemType());
        telemetry.addData("System Status", odometry.getSystemStatus());
        telemetry.addData("Update Rate", String.format("%.1f Hz", odometry.getUpdateFrequency()));
        
        // Position information
        telemetry.addData("", "");
        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("Robot Position", odometry.getFormattedPosition());
        telemetry.addData("X Position", String.format("%.2f inches", odometry.getX()));
        telemetry.addData("Y Position", String.format("%.2f inches", odometry.getY()));
        telemetry.addData("Heading", String.format("%.1f degrees", odometry.getHeadingDegrees()));
        
        // Velocity (Pinpoint only)
        if (odometry.isPinpointSystem()) {
            double[] velocity = odometry.getVelocity();
            telemetry.addData("", "");
            telemetry.addData("=== VELOCITY ===", "");
            telemetry.addData("X Velocity", String.format("%.2f in/sec", velocity[0]));
            telemetry.addData("Y Velocity", String.format("%.2f in/sec", velocity[1]));
            telemetry.addData("Angular Velocity", String.format("%.2f deg/sec", Math.toDegrees(velocity[2])));
        }
        
        // Encoder data
        telemetry.addData("", "");
        telemetry.addData("=== ENCODER DATA ===", "");
        telemetry.addData("Raw Positions", odometry.getEncoderPositions());
        telemetry.addData("Converted Values", odometry.getEncoderInches());
        
        // Debug information
        if (debugMode) {
            telemetry.addData("", "");
            telemetry.addData("=== DEBUG INFO ===", "");
            
            if (odometry.isTraditionalSystem()) {
                telemetry.addData("Track Width", String.format("%.3f inches", odometry.getTrackWidth()));
                telemetry.addData("Horizontal Offset", String.format("%.3f inches", odometry.getHorizontalOffset()));
            }
            
            // System-specific debug info is handled by odometry.addTelemetry()
            odometry.addTelemetry();
        }
        
        // DECODE field reference
        telemetry.addData("", "");
        telemetry.addData("=== DECODE FIELD ===", "");
        telemetry.addData("Field Size", "141\" x 141\"");
        telemetry.addData("Origin (0,0)", "Field Center");
        telemetry.addData("Red Alliance", "Y = -52.5\" (left side)");
        telemetry.addData("Blue Alliance", "Y = +52.5\" (right side)");
        telemetry.addData("Audience", "X = +70.5\" (positive direction)");
        
        // Example field positions
        telemetry.addData("", "");
        telemetry.addData("=== EXAMPLE POSITIONS ===", "");
        telemetry.addData("Red Start", "(-48, -48, 0°)");
        telemetry.addData("Blue Start", "(-48, +48, 0°)");
        telemetry.addData("Center", "(0, 0, 0°)");
        telemetry.addData("Audience Side", "(+60, 0, 0°)");
        
        // Navigation helpers
        double distanceFromCenter = odometry.getDistanceToPoint(0, 0);
        double angleToCenter = odometry.getAngleToPoint(0, 0);
        
        telemetry.addData("", "");
        telemetry.addData("=== NAVIGATION ===", "");
        telemetry.addData("Distance to Center", String.format("%.2f inches", distanceFromCenter));
        telemetry.addData("Angle to Center", String.format("%.1f degrees", angleToCenter));
        
        // Controls reminder
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Reset to origin");
        telemetry.addData("B", "DECODE field position");
        telemetry.addData("X", "Debug: " + (debugMode ? "ON" : "OFF"));
        if (odometry.isPinpointSystem()) {
            telemetry.addData("Y", "Recalibrate IMU");
        }
    }
}