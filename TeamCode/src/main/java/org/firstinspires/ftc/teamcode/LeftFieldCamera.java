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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;


@Autonomous(name="LeftFieldCamera", group="Robot")

public class LeftFieldCamera extends LinearOpMode {

    //Declaring DcMotor Objects
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightHexLift = null;
    private DcMotor leftHexLift = null;
    private DcMotor frontArm = null;

    double ticksPR = 537.6;
    double WHEEL_DIAMETER_MILLIMETERS = 96.8;
    double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MILLIMETERS*0.039;
    double COUNTS_PER_INCH = ticksPR / (WHEEL_DIAMETER_INCHES * 3.1415);//11.3477 pules per inch with 96mm

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
    double liftPower;

    int globalSleep = 5;

    double globalCorrection = 1.084;

    // Declare the webcam and pipeline objects
    private OpenCvCamera webcam;
    private SampleDetectionPipeline pipeline;

    //Main OpMode Code \/
    @Override
    public void runOpMode() {

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

        // Initialize the webcam and pipeline.
        //cameraMonitorViewId is the container we will use to display the camera stream
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Set the webcam object to the instance of the webcam
        //Make sure to change the name of the webcam to the one you are using("Webcam 1" is the default name).
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initializes our custom pipeline. (Make sure SampleDetectionPipeline is available.)
        pipeline = new SampleDetectionPipeline();//Create a new SampleDetectionPipeline object and names it pipeline.
        webcam.setPipeline(pipeline);//Sets the pipeline to to use.

        // Open the camera asynchronously. Asynchronous means that the camera will start opening in the background and your program will continue executing.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override//Override the onOpened method. Overriding is the action of defining a new behavior for a method in the subclass.
            public void onOpened() {
                // Start streaming at 320x240. Adjust resolution as needed.
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//Starts streaming the camera feed.
            }
            @Override
            //On error method is called when an error occurs.
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Drive CPI = ", COUNTS_PER_INCH); //Counts Per Inch is displayed for debugging purposes.
        telemetry.addData("Status", "Ready to run");    //Displays "Ready to run" on telemetry
        telemetry.update();

        //Close Pincher to Ready Robot
        closePinch();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Optional: Give the pipeline a moment to update
        sleep(1000);

        //RUNNING AUTONOMOUS STEPS//
        //Making sure pincher is closed
        closePinch();

        //Making sure arm is all the way the back
        frontArm.setPower(-0.5);
        sleep(250);

        //Following is Main the Autonomous Steps:

        //Step 1: Lift the arm up and move to basket
        //right(driveSpeed, 8, 2);//Move to the right to be away from the wall
        //turnLeft(driveSpeed, 8.5, 2);//Turn to face the basket
        //right(driveSpeed, 15, 3);//Move to face the basket
        rightForward(driveSpeed, 31, 2.0);
        turnLeft(driveSpeed, 8.5, 1.9);
        //forward(driveSpeed, 10, 2);//move to basket
        //liftUp(1.0, 730, 4);//Lift arm up to be ready to drop sample in higher basket
        forwardWithLift(0.2, 12, 1.5, 720, 0, 3.0);
        leftHexLift.setPower(-0.3);
        rightHexLift.setPower(-0.3);
        forward(driveSpeed, 3, 1);
        moveArm(0.6, 0.2);//Move arm out to drop sample
        sleep(50);//wait for arm to move
        openPinch();//Open pinch to drop sample
        frontArm.setPower(-0.5);//Set arm power to move back and keep it there

        //Step 2: Move into position to pick up sample
        backward(driveSpeed, 8, 1);//move away from basket
        liftDown(1.0, 520, 2);//bring lift down
        turnRight(0.65, 30, 2);//Turn to face sample
        right(driveSpeed, 3.8, 1);//Small adjustment to face sample
        moveArm(0.7, 0.4);//Move arm out to pick up sample
        forward(driveSpeed, 6.7, 1);//Move to sample
        openPinch();
        alignSample(2, 45, 0.5, 0.5, 0.8);//Align with sample using camera
        openPinch();
        liftDown(1.0, 115, 3);//Bring lift down to pick up sample

        //Step 3: Pick up sample and move to basket
        sleep(100);//wait for robot to stop moving
        closePinch();//close pinch to pick up sample
        sleep(250);//wait for pinch to close
        moveArm(-1.0, 1.0);//Move arm back in
        frontArm.setPower(-0.7);//Set arm power to move back and keep it there
        turnLeft(0.75, 32, 1.5);//turn to face basket
        left(driveSpeed, 3, 1);//line up with basket
        //liftUp(1.0, 710, 3);//Lift arm up to drop sample
        //forward(driveSpeed, 14, 3);//Move to basket
        forwardWithLift(0.2, 15, 0, 730, 0, 2.8);

        //Step 4: Drop sample and move to next sample
        sleep(100);//wait for robot to stop moving
        moveArm(0.6, 0.3);//Move arm out to drop sample
        sleep(50);//wait for arm to move
        openPinch();//Open pinch to drop sample
        frontArm.setPower(-0.5);//Set arm power to move back and keep it there
        //backward(driveSpeed, 5, 1);//Move back from basket
        //liftDown(1.0, 500, 4);//Bring lift down
        backwardWithLift(0.3, 5, 0, -720, 0, 3);
        turnRight(0.75, 29.5, 3);//Turn to face next sample

        //Stop Motors
        stopDrive();
        //END STEPS//*/

        telemetry.addData("Auto", "Complete");
        telemetry.update();
        sleep(1000);
        //END OF PROGRAM//

    }

    //circumference = 11.87 inches

    /*Feedback Control Functions*/
    public void alignSample(double timeout, int margin, double speed, double increment, double inctimeout) {
        // Vision-based alignment: adjust lateral position until the sample is centered
        telemetry.addData("Vision", "Aligning with sample...");
        telemetry.update();

        runtime.reset();

        // Loop until the sample is centered within a tolerance(10 pixels)
        while (opModeIsActive()&& runtime.seconds() < timeout) {
            openPinch();
            Point sampleCenter = pipeline.getSampleCenter();  // Gets the center coordinates of the sample on the camera.
            if (sampleCenter != null) {
                double errorX = sampleCenter.x - 160;  // 160 = center of a 320-pixel wide image
                double errorY = sampleCenter.y - 100;  // 120 = center of a 240-pixel tall image
                telemetry.addData("Sample Center", sampleCenter.toString());
                telemetry.addData("Error X", errorX);
                telemetry.addData("Error Y", errorY);
                telemetry.update();

                // Check if sample is aligned (within 10 pixels)
                if (Math.abs(errorX) < margin && Math.abs(errorY) < margin) {
                    telemetry.addData("Status", "Aligned!");
                    telemetry.update();
                    break;  // Exit the loop when aligned
                }

                // Adjust X position: if errorX is positive, sample is to the right, so move left; if negative, move right.
                if (errorX > 0) {
                    // Move left slightly. Adjust speed, distance, and timeout.
                    left(speed, increment, inctimeout);
                } else {
                    // Move right slightly.
                    right(speed, increment, inctimeout);
                }
                //Adjust Y Position:
                if (errorY > 0) {
                    //Move forward slightly
                    forward(speed, increment, inctimeout);
                } else {
                    //Move backward slightly
                    backward(speed, increment, inctimeout);
                }
            } else {
                //If there is no sample detected then
                telemetry.addData("Vision", "Sample not detected");
                telemetry.update();
                sleep(250);  // Short delay between adjustments
            }

        }
    }

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

    public void rightForward(double speed, double inches, double timeout) {

        int target = (int)(inches * COUNTS_PER_INCH * (globalCorrection));
        runDrive(target, 0, 0, target, speed, timeout);
    }

    public void forwardWithLift(double speed, int inches, double driveDelay, int ticks, double liftDelay, double timeout){

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
                    (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                            leftBackDrive.isBusy() || rightBackDrive.isBusy() ||
                            leftHexLift.isBusy() || rightHexLift.isBusy())) {

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
                telemetry.addData("Lift",  "Running to %7d", liftTarget);
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

    public void backwardWithLift(double speed, int inches, double driveDelay, int ticks, double liftDelay, double timeout){

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
                    (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                            leftBackDrive.isBusy() || rightBackDrive.isBusy() ||
                            leftHexLift.isBusy() || rightHexLift.isBusy())) {

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
                telemetry.addData("Lift",  "Running to %7d", liftTarget);
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

    public void forward(double speed, double inches, double timeout) {
        // For forward movement, all motors move the same direction.
        int target = (int)(inches * COUNTS_PER_INCH * (globalCorrection - 0.05));
        runDrive(target, target, target, target, speed, timeout);
    }

    public void backward(double speed, double inches, double timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        runDrive(-target, -target, -target, -target, speed, timeout);
    }

    public void right(double speed, double inches, double timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For right strafe the front left and back right go forward; the other two go in reverse.
        runDrive(target, -target, -target, target, speed, timeout);
    }

    public void left(double speed, double inches, double timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        runDrive(-target, target, target, -target, speed, timeout);
    }

    public void turnRight(double speed, double inches, double timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        // For turning right, the left motors move forward and right motors move backward.
        runDrive(target, -target, target, -target, speed, timeout);
    }

    public void turnLeft(double speed, double inches, double timeout) {
        int target = (int)(inches * COUNTS_PER_INCH * globalCorrection);
        runDrive(-target, target, -target, target, speed, timeout);
    }


    public void liftDown(double speed, int ticks, double timeout){

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

    public void liftUp(double speed, int ticks, double timeout){

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
    private void openPinch() {
        rightPinch.setPosition(0.35);
        leftPinch.setPosition(0.20);
    }

    private void closePinch() {
        rightPinch.setPosition(0.0);
        leftPinch.setPosition(0.0);
    }

    //Moves the arm for a specified time with a specified power
    private void moveArm(double power, double time){
        frontArm.setPower(power);
        sendPower();
        waitWhile(time);
        frontArm.setPower(0.0);
    }
}



