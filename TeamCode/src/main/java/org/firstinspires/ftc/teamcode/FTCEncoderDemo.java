package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * FTC Encoder Best Practices Demonstration OpMode
 * 
 * This OpMode demonstrates proper encoder usage following FTC guidelines:
 * 1. Reset encoders to zero at OpMode start
 * 2. Use RUN_WITHOUT_ENCODER after reset (doesn't disable encoders)
 * 3. Read encoder values in different units (counts, revolutions, inches, degrees)
 * 4. Proper encoder value calculations using CPR (Counts Per Revolution)
 * 
 * Based on official FTC documentation and best practices.
 */
@TeleOp(name = "FTC Encoder Demo", group = "Demo")
public class FTCEncoderDemo extends LinearOpMode {
    
    // Motor for demonstration - adjust name to match your robot configuration
    private DcMotor testMotor;
    
    // Robot constants - REPLACE WITH YOUR ROBOT'S VALUES
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    // For GoBILDA 312 RPM motor
    private static final double DRIVE_GEAR_REDUCTION = 1.0;      // No external gearing
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // 4 inch wheels
    
    // Calculated constants
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                   (WHEEL_DIAMETER_INCHES * Math.PI);
    
    @Override
    public void runOpMode() {
        // Find a motor in the hardware map - ADJUST NAME TO MATCH YOUR CONFIG
        testMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        
        // Reset the motor encoder so that it reads zero ticks (FTC best practice)
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        // RUN_WITHOUT_ENCODER does NOT disable the encoder, it just disables built-in velocity control
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set motor direction and brake behavior
        testMotor.setDirection(DcMotor.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized - FTC Encoder Demo");
        telemetry.addData("Instructions", "Use gamepad to drive motor and see encoder values");
        telemetry.addData("Controls", "Left stick Y = motor power");
        telemetry.addData("Encoder CPR", COUNTS_PER_MOTOR_REV);
        telemetry.addData("Wheel Diameter", WHEEL_DIAMETER_INCHES + " inches");
        telemetry.addData("Counts Per Inch", String.format("%.2f", COUNTS_PER_INCH));
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Control motor with gamepad
            double power = -gamepad1.left_stick_y; // Negative because Y stick is inverted
            testMotor.setPower(power);
            
            // Get the current position of the motor (in encoder counts)
            int position = testMotor.getCurrentPosition();
            
            // Calculate revolutions (total rotations)
            double revolutions = position / COUNTS_PER_MOTOR_REV;
            
            // Calculate total angle (can exceed 360 degrees)
            double angle = revolutions * 360.0;
            
            // Calculate normalized angle (0-360 degrees)
            double angleNormalized = angle % 360.0;
            if (angleNormalized < 0) angleNormalized += 360.0; // Handle negative values
            
            // Calculate linear distance (for wheels/spools)
            double distance = (position / COUNTS_PER_INCH);
            
            // Display all encoder information
            telemetry.addData("Motor Power", String.format("%.2f", power));
            telemetry.addData("Encoder Position (counts)", position);
            telemetry.addData("Encoder Revolutions", String.format("%.3f", revolutions));
            telemetry.addData("Encoder Angle (Total)", String.format("%.1f°", angle));
            telemetry.addData("Encoder Angle (0-360°)", String.format("%.1f°", angleNormalized));
            telemetry.addData("Linear Distance", String.format("%.2f inches", distance));
            
            // Additional information
            telemetry.addData("", "--- Debug Info ---");
            telemetry.addData("Counts Per Revolution", COUNTS_PER_MOTOR_REV);
            telemetry.addData("Counts Per Inch", String.format("%.2f", COUNTS_PER_INCH));
            telemetry.addData("Current Run Mode", testMotor.getMode().toString());
            
            // Instructions
            telemetry.addData("", "--- Instructions ---");
            telemetry.addData("Info", "Turn motor 1 full revolution (360°)");
            telemetry.addData("Info", "Encoder should show ~" + (int)COUNTS_PER_MOTOR_REV + " counts");
            telemetry.addData("Info", "Distance = circumference * revolutions");
            telemetry.addData("Expected Distance/Rev", String.format("%.2f inches", 
                WHEEL_DIAMETER_INCHES * Math.PI));
            
            telemetry.update();
            
            // Small delay to make telemetry readable
            sleep(50);
        }
        
        // Stop motor when OpMode ends
        testMotor.setPower(0);
    }
}