package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * FTC Encoder RUN_TO_POSITION Demonstration OpMode
 * 
 * This OpMode demonstrates using encoders with RUN_TO_POSITION mode
 * following FTC best practices:
 * 1. Reset encoders to zero at start
 * 2. Set target positions in encoder counts
 * 3. Use RUN_TO_POSITION mode for precise movement
 * 4. Monitor position and completion status
 * 
 * This is useful for arms, lifts, and precise drivetrain movements.
 */
@Autonomous(name = "FTC Encoder RUN_TO_POSITION Demo", group = "Demo")
public class FTCEncoderRunToPositionDemo extends LinearOpMode {
    
    // Motors for demonstration - adjust names to match your robot configuration
    private DcMotor armMotor;
    private DcMotor driveMotor;
    
    // Robot constants - REPLACE WITH YOUR ROBOT'S VALUES
    private static final double ARM_COUNTS_PER_REV = 537.7;      // For GoBILDA 312 RPM motor
    private static final double DRIVE_COUNTS_PER_REV = 537.7;    // For GoBILDA 312 RPM motor
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // 4 inch wheels
    private static final double DRIVE_COUNTS_PER_INCH = (DRIVE_COUNTS_PER_REV) /
                                                         (WHEEL_DIAMETER_INCHES * Math.PI);
    
    // Arm positions in encoder counts
    private static final int ARM_DOWN_POSITION = 0;              // Starting position
    private static final int ARM_UP_POSITION = 1000;            // Raised position (adjust for your robot)
    private static final int ARM_MIDDLE_POSITION = 500;         // Middle position
    
    // Drive distances in encoder counts
    private static final int DRIVE_12_INCHES = (int)(12 * DRIVE_COUNTS_PER_INCH);
    private static final int DRIVE_24_INCHES = (int)(24 * DRIVE_COUNTS_PER_INCH);
    
    @Override
    public void runOpMode() {
        // Initialize motors - ADJUST NAMES TO MATCH YOUR CONFIG
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        driveMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        
        // Reset encoders (FTC best practice)
        resetEncoders();
        
        // Set motor directions and brake behavior
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        driveMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized - RUN_TO_POSITION Demo");
        telemetry.addData("Arm Positions", String.format("Down:%d Mid:%d Up:%d", 
            ARM_DOWN_POSITION, ARM_MIDDLE_POSITION, ARM_UP_POSITION));
        telemetry.addData("Drive Distances", String.format("12\":%d 24\":%d counts", 
            DRIVE_12_INCHES, DRIVE_24_INCHES));
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Sequence of movements demonstrating RUN_TO_POSITION
            
            // Step 1: Raise arm to middle position
            telemetry.addData("Step", "1 - Raising arm to middle position");
            telemetry.update();
            runMotorToPosition(armMotor, ARM_MIDDLE_POSITION, 0.5, 3.0);
            sleep(500);
            
            // Step 2: Drive forward 12 inches
            telemetry.addData("Step", "2 - Driving forward 12 inches");
            telemetry.update();
            runMotorToPosition(driveMotor, DRIVE_12_INCHES, 0.6, 5.0);
            sleep(500);
            
            // Step 3: Raise arm to full up position
            telemetry.addData("Step", "3 - Raising arm to full up position");
            telemetry.update();
            runMotorToPosition(armMotor, ARM_UP_POSITION, 0.7, 3.0);
            sleep(500);
            
            // Step 4: Drive forward another 12 inches (total 24)
            telemetry.addData("Step", "4 - Driving forward another 12 inches");
            telemetry.update();
            runMotorToPosition(driveMotor, DRIVE_24_INCHES, 0.6, 5.0);
            sleep(500);
            
            // Step 5: Lower arm back to down position
            telemetry.addData("Step", "5 - Lowering arm to down position");
            telemetry.update();
            runMotorToPosition(armMotor, ARM_DOWN_POSITION, 0.5, 3.0);
            sleep(500);
            
            // Step 6: Drive back to start
            telemetry.addData("Step", "6 - Driving back to start position");
            telemetry.update();
            runMotorToPosition(driveMotor, 0, 0.6, 5.0);
            sleep(500);
            
            telemetry.addData("Status", "Demo Complete!");
            telemetry.addData("Final Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Final Drive Position", driveMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    
    /**
     * Reset all motor encoders (FTC best practice)
     */
    private void resetEncoders() {
        // Stop and reset encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Turn motors back on (required after STOP_AND_RESET_ENCODER)
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addData("Encoders", "Reset to zero");
        telemetry.update();
    }
    
    /**
     * Run a motor to a specific position using RUN_TO_POSITION mode
     * @param motor The motor to move
     * @param targetPosition Target position in encoder counts
     * @param power Maximum power (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to wait for completion
     */
    private void runMotorToPosition(DcMotor motor, int targetPosition, double power, double timeoutSeconds) {
        // Set target position BEFORE setting mode (important!)
        motor.setTargetPosition(targetPosition);
        
        // Set motor to RUN_TO_POSITION mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Set maximum power
        motor.setPower(Math.abs(power));
        
        // Wait for motor to reach position or timeout
        double startTime = getRuntime();
        while (opModeIsActive() && motor.isBusy() && 
               (getRuntime() - startTime) < timeoutSeconds) {
            
            // Display progress
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Error", targetPosition - motor.getCurrentPosition());
            telemetry.addData("Motor Busy", motor.isBusy());
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Time Remaining", String.format("%.1f sec", 
                timeoutSeconds - (getRuntime() - startTime)));
            telemetry.update();
            
            idle(); // Allow other processes to run
        }
        
        // Stop motor
        motor.setPower(0);
        
        // Switch back to RUN_WITHOUT_ENCODER for next operation
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Report completion
        int finalPosition = motor.getCurrentPosition();
        int error = targetPosition - finalPosition;
        telemetry.addData("Movement Complete", "");
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Final Position", finalPosition);
        telemetry.addData("Error", error + " counts");
        telemetry.addData("Accuracy", String.format("%.1f%%", 
            100.0 * (1.0 - Math.abs(error) / (double)Math.abs(targetPosition))));
        telemetry.update();
    }
}
