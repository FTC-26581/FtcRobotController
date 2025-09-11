/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;//Package Declaration

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;//Include the Autonomous Library
import com.qualcomm.robotcore.hardware.Servo;//Include the Servo Library
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;//Include the LinearOpMode Library
import com.qualcomm.robotcore.hardware.DcMotor;//Include the DcMotor Library
import com.qualcomm.robotcore.util.ElapsedTime;//Include the Elapsed Time Library
import com.qualcomm.hardware.bosch.BNO055IMU;//Include the IMU Library
import com.qualcomm.robotcore.util.Range;//Include the Range Library

//Import the necessary classes for the IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="LeftFieldFeedback", group="Robot")//Register the class as an Autonomous program and set the name and group

//Class Declaration
public class LeftFieldFeedback extends LinearOpMode {

    //PIDController pidController = new PIDController(0.1, 0.1, 0.05);

    //Declare IMU
    private BNO055IMU imu;


    //Declaring DcMotor Objects//

    //Declare Drive Motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //Declare Lift Motors
    private DcMotor rightHexLift = null;
    private DcMotor leftHexLift = null;

    private DcMotor frontArm = null;//Declare Arm Motor

    //Declaring and setting variables for encoder counts math
    double ticksPR = 537.6;
    double WHEEL_DIAMETER_MILLIMETERS = 96.8;
    double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MILLIMETERS*0.039;
    double COUNTS_PER_INCH = ticksPR / (WHEEL_DIAMETER_INCHES * 3.1415);//11.3477 pules per inch with 96mm

    //Declare Global Speed Variables
    double driveSpeed = 0.6;
    
    //Pincher servos
    Servo rightPinch;
    Servo leftPinch;
    
    double pinchPos;
    double pinch2Pos;
    
    //Declare Power Variables for Mechanum Drive
    static double leftFrontPower;
    static double rightFrontPower;
    static double leftBackPower;
    static double rightBackPower;

    //Setup Runtime
    private final ElapsedTime     runtime = new ElapsedTime();
    
    //Declare Mechanum Drive Math Variables
    double max;
    double axial;
    double lateral;
    double yaw;
    
    //Lift power
    double liftPower = 0.0;//Set Lift Power to 0 to start
    
    int globalSleep = 250;//Set Global Sleep Time to 250ms
    
    double globalCorrection = 1.084;//Set Global Correction to 1.084.
    //globalCorrection is used to correct the distance the robot moves. In this case, the robot moves 8.4% more than it should

    //Main OpMode Code \/
    @Override
    public void runOpMode() {

        // Setup IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; // use degrees for easier math
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // if available
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Initialize the IMU (the name "imu" must match your configuration)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the IMU to calibrate
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("IMU Status", "Calibrating...");
            telemetry.update();
            sleep(50);
        }
        telemetry.addData("IMU Status", "Calibrated");
        telemetry.update();

        // Initialize the drive system variables.
        
        //Dc Motor Mapping
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        
        rightHexLift  = hardwareMap.get(DcMotor.class, "rightLift");
        leftHexLift  = hardwareMap.get(DcMotor.class, "leftLift");
        
        frontArm  = hardwareMap.get(DcMotor.class, "frontArm");
        
        //Servo Mapping
        rightPinch = hardwareMap.get(Servo.class, "rightPinch");
        leftPinch = hardwareMap.get(Servo.class, "leftPinch");
        
        
        //Set Power Behaviors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightHexLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHexLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Motor Directions//
        
        //Drive Train Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        //Arm Motor Direction
        frontArm.setDirection(DcMotor.Direction.FORWARD);
        
        //Servo Directions
        rightPinch.setDirection(Servo.Direction.FORWARD);
        leftPinch.setDirection(Servo.Direction.REVERSE);
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Drive CPI = ", COUNTS_PER_INCH);//Display the counts per inch for the drive motors encoders in telemetry
        telemetry.addData("Status", "Ready to run");    //Display the status of the robot in telemetry
        telemetry.update();
        
        //Close Pincher to Ready Robot
        closePinch();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        
        //Making sure pincher is closed
        closePinch();
        
        //Making sure arm is all the way the back
        frontArm.setPower(-0.5);
        sleep(250);//Give arm time to move
        
        //Following is the Autonomous Steps:
        
        liftUp(1.0, 640, 4);//Raise Lift to highest position
        right(driveSpeed, 6, 2);//Move away from wall
        turnLeft(driveSpeed, 8.5, 2);//Turn to face basket
        right(driveSpeed, 13.5, 3);//Move to align with basket
        forward(driveSpeed, 18, 5);//Move to basket
        moveArm(0.2, 0.1);//Move arm out to drop off sample
        sleep(150);//Give arm time to move
        openPinch();//Open Pincher to drop off sample
        frontArm.setPower(-0.5);//Move arm back in
        
        backward(driveSpeed, 9, 3);//Move away from basket
        liftDown(1.0, 400, 4);//Lower lift to position to pick up sample
        turnRight(driveSpeed, 29.8, 3);//Turn to face sample
        left(driveSpeed, 0.3, 1);//Move to align with sample
        moveArm(0.3, 0.6);//Move arm out to pick up sample
        forward(driveSpeed, 8.2, 3);//Move to pick up sample
        backward(driveSpeed, 2.1, 2);//Small adjustment to pick up sample
        sleep(250);//Give robot time to stop moving
        closePinch();//Close Pincher to pick up sample
        sleep(250);//Give pinchers time to pick up sample
        moveArm(-0.8, 0.8);//Move arm back in
        frontArm.setPower(-0.5);//Keep arm in by setting power to -0.5
        turnLeft(driveSpeed, 33, 3);//Turn to face basket
        left(driveSpeed, 3, 2);//Move to align with basket
        liftUp(1.0, 400, 4);//Raise lift to highest position
        forward(driveSpeed, 14, 3);//Move to basket
        sleep(150);//Give robot time to stop moving
        moveArm(0.2, 0.1);//Move arm out to drop off sample
        sleep(150);//Give arm time to move
        openPinch();//Open Pincher to drop off sample
        frontArm.setPower(-0.5);//Move arm back in
        backward(driveSpeed, 5, 1);//Move away from basket
        liftDown(1.0, 640, 4);//Lower lift to lowest position
        turnRight(driveSpeed, 29.5, 3);//Turn to face next sample
        
        //Stop Motors
        stopDrive();
        //END STEPS//

        telemetry.addData("Auto", "Complete");
        telemetry.update();
        sleep(1000);
        //END OF PROGRAM//
        
    }

    //circumference = 11.87 inches 

    /*Feedback Control Functions*/

    /**
     * Drives the robot by setting target positions for each drive motor.
     *
     * @param lfTarget  target position for left front motor
     * @param rfTarget  target position for right front motor
     * @param lbTarget  target position for left back motor
     * @param rbTarget  target position for right back motor
     * @param speed     motor power (0 to 1)
     * @param timeout   timeout (in seconds) before giving up
     */
    private void runDrive(int lfTarget, int rfTarget, int lbTarget, int rbTarget, double speed, double timeout) {
        // Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        leftFrontDrive.setTargetPosition(lfTarget);
        rightFrontDrive.setTargetPosition(rfTarget);
        leftBackDrive.setTargetPosition(lbTarget);
        rightBackDrive.setTargetPosition(rbTarget);

        // Set mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors
        runtime.reset();
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait until motors reach target or timeout is reached.
        while (opModeIsActive() && runtime.seconds() < timeout &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("Target", "LF: %d | RF: %d | LB: %d | RB: %d", lfTarget, rfTarget, lbTarget, rbTarget);
            telemetry.addData("Current", "LF: %d | RF: %d | LB: %d | RB: %d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
        stopDrive();  // Stop all motors.
        sleep(globalSleep);
    }

    //Forward with Lift Function: Moves the robot forward while raising or lowering the lift.
    public void forwardWithLift(double speed, int inches, double driveDelay, int ticks, double liftDelay, int timeout){

        //Stop and Reset Encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target;
        int liftTarget;

        if(opModeIsActive()){

            //Set Target Variables
            target = (int)(inches*COUNTS_PER_INCH*(globalCorrection -0.05));
            liftTarget = ticks;

            //Set Target Positions
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(target);

            leftHexLift.setTargetPosition(liftTarget);
            rightHexLift.setTargetPosition(liftTarget);

            //Set RunMode for Encoders
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Reset Runtime
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < timeout &&
                    ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) ||
                            (leftHexLift.isBusy() || rightHexLift.isBusy()))) {

                if(runtime.seconds() >= driveDelay){
                    leftFrontDrive.setPower(speed);
                    rightFrontDrive.setPower(speed);
                    leftBackDrive.setPower(speed);
                    rightBackDrive.setPower(speed);
                }

                if(runtime.seconds() >= liftDelay){
                    leftHexLift.setPower(1);
                    rightHexLift.setPower(1);
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Lift",  "Running to %7d", target);
                telemetry.addData("Lift",  "Lift-at %7d :%7d", leftHexLift.getCurrentPosition(), rightHexLift.getCurrentPosition());
                telemetry.update();
            }

            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            leftHexLift.setPower(0);
            rightHexLift.setPower(0);

            sleep(globalSleep);

        }

    }

    //Backward with Lift Function: Moves the robot backward while raising or lowering the lift.
    public void backwardWithLift(double speed, int inches, double driveDelay, int ticks, double liftDelay, int timeout){

        //Stop and Reset Encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target;
        int liftTarget;

        if(opModeIsActive()){

            //Set Target Variables
            target = (int)(inches*COUNTS_PER_INCH*(globalCorrection -0.05));
            liftTarget = ticks;

            //Set Target Positions
            leftFrontDrive.setTargetPosition(-target);
            rightFrontDrive.setTargetPosition(-target);
            leftBackDrive.setTargetPosition(-target);
            rightBackDrive.setTargetPosition(-target);

            leftHexLift.setTargetPosition(-liftTarget);
            rightHexLift.setTargetPosition(-liftTarget);

            //Set RunMode for Encoders
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Reset Runtime
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < timeout &&
                    ((leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) ||
                            (leftHexLift.isBusy() || rightHexLift.isBusy()))) {

                if(runtime.seconds() >= driveDelay){
                    leftFrontDrive.setPower(speed);
                    rightFrontDrive.setPower(speed);
                    leftBackDrive.setPower(speed);
                    rightBackDrive.setPower(speed);
                }

                if(runtime.seconds() >= liftDelay){
                    leftHexLift.setPower(1);
                    rightHexLift.setPower(1);
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Lift",  "Running to %7d", target);
                telemetry.addData("Lift",  "Lift-at %7d :%7d", leftHexLift.getCurrentPosition(), rightHexLift.getCurrentPosition());
                telemetry.update();
            }

            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            leftHexLift.setPower(0);
            rightHexLift.setPower(0);

            sleep(globalSleep);

        }

    }

    public void forward(double speed, double inches, int timeout) {
        // For forward movement, all motors move the same direction.
        int target = (int)(inches * COUNTS_PER_INCH * (globalCorrection - 0.05));
        runDrive(target, target, target, target, speed, timeout);
    }

    public void backward(double speed, double inches, int timeout) {
        //For backward movement, all motors move in reverse.
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        runDrive(-target, -target, -target, -target, speed, timeout);
    }

    public void right(double speed, double inches, int timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For right strafe the front left and back right go forward; the other two go in reverse.
        runDrive(target, -target, -target, target, speed, timeout);
    }

    public void left(double speed, double inches, int timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For left strafe the front right and back left go forward; the other two go in reverse.
        runDrive(-target, target, target, -target, speed, timeout);
    }

    public void turnRight(double speed, double inches, int timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For turning right, the left motors move forward and right motors move backward.
        runDrive(target, -target, target, -target, speed, timeout);
    }

    public void turnLeft(double speed, double inches, int timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For turning left, the right motors move forward and left motors move backward.
        runDrive(-target, target, -target, target, speed, timeout);
    }
    
    public void liftUp(double speed, int ticks, int timeout){
        
        leftHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = ticks;
            
            leftHexLift.setTargetPosition(target);
            rightHexLift.setTargetPosition(target);
            
            leftHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftHexLift.setPower(speed);
            rightHexLift.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftHexLift.isBusy() && rightHexLift.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Lift-at %7d :%7d", leftHexLift.getCurrentPosition(), rightHexLift.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftHexLift.setPower(0);
            rightHexLift.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void liftDown(double speed, int ticks, int timeout){
        
        leftHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = ticks;
            
            leftHexLift.setTargetPosition(-target);
            rightHexLift.setTargetPosition(-target);
            
            leftHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightHexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftHexLift.setPower(speed);
            rightHexLift.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftHexLift.isBusy() && rightHexLift.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Lift-at %7d :%7d", leftHexLift.getCurrentPosition(), rightHexLift.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftHexLift.setPower(0);
            rightHexLift.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }

    //Gyro Turn Function: Turns the robot using the gyro sensor.
    public void turnToAngle(double speed, double targetAngle) {
        double currentAngle = getGyroAngle();
        while (opModeIsActive() && Math.abs(targetAngle - currentAngle) > 1) {
            double error = targetAngle - currentAngle;

            // Normalize error to be within -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Check if within an acceptable error margin
            if (Math.abs(error) < 1) break;

            double turnPower = Range.clip(error * 0.01, -speed, speed); // Proportional control

            leftFrontDrive.setPower(turnPower);
            rightFrontDrive.setPower(-turnPower);
            leftBackDrive.setPower(turnPower);
            rightBackDrive.setPower(-turnPower);
            currentAngle = getGyroAngle();

            telemetry.addData("Error", error);
            telemetry.update();
        }
        stopDrive();
    }

    private double getGyroAngle() {
        // Read the heading (rotation about the Z-axis)
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

        telemetry.addData("Heading", heading);
        telemetry.update();
        return heading;
    }

    
    /*Drive Control and Calculation Functions*/
    //Calculate Power for Motors using the axial, lateral & yaw variables.
    public void calcPower() {
        
        // Combine the control requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower  = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower   = axial - lateral + yaw;
        rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
    }
    
    //Send The Calculated Power from calcPower Function.
    public void sendPower() {
        calcPower();
        
        rightPinch.setPosition(pinch2Pos);
        leftPinch.setPosition(pinchPos);
        
        //Send Power for Lift Motors
        rightHexLift.setPower(liftPower);
        leftHexLift.setPower(liftPower);
        
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    
    //Set Power to Zero for all Motors
    private void stopDrive(){
        axial=0;
        lateral=0;
        yaw=0;
        liftPower = 0;
        sendPower();
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    
    /*Time Functions*/
    //Function to wait specified time in seconds without stopping robot.
    private void waitWhile(double time){
        
        runtime.reset();
        
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", " %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    
    /*Pincher Functions*/
    //Opens the pincher
    private void openPinch(){
        pinchPos = 0.20;
        pinch2Pos = 0.35;
        sendPower();
    }
    
    //Closes the pincher
    private void closePinch(){
        pinchPos = 0;
        pinch2Pos = 0;
        sendPower();
    }
    
    
    /*Timed Movement Functions*/
    private void moveArm(double power, double time){
        frontArm.setPower(power);
        sendPower();
        waitWhile(time);
        frontArm.setPower(0.0);
    }
}



