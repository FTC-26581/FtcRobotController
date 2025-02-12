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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="LeftFieldFeedback", group="Robot")

public class LeftFieldFeedback extends LinearOpMode {

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
    private ElapsedTime     runtime = new ElapsedTime();
    
    //Declare Mechanum Drive Math Variables
    double max;
    double axial;
    double lateral;
    double yaw;
    
    //Liftpower
    double liftPower = 0.0;
    
    int globalSleep = 250;
    
    double globalCorection = 1.084;

    //Main Opmode Code \/
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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Drive CPI = ", COUNTS_PER_INCH);
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        
        //Close Pincher to Ready Robot
        closePinch();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        
        //Making sure pincher is closed
        closePinch();
        
        //Making sure arm is all the way the back
        frontArm.setPower(-0.5);
        sleep(250);
        
        //Following is the Autonomous Steps:
        //sleep(29000);//For Testing Purposes
        
        liftUp(1.0, 1390, 4);
        right(driveSpeed, 6, 2);
        turnLeft(driveSpeed, 8.5, 2);
        right(driveSpeed, 13.5, 3);
        forward(driveSpeed, 18, 5);
        moveArm(0.2, 0.1);
        sleep(150);
        openPinch();
        frontArm.setPower(-0.5);
        
        backward(driveSpeed, 9, 3);
        liftDown(1.0, 1255, 4);
        turnRight(driveSpeed, 29.8, 3);
        left(driveSpeed, 0.3, 1);
        moveArm(0.3, 0.6);
        forward(driveSpeed, 8.2, 3);
        backward(driveSpeed, 2.1, 2);
        sleep(250);
        closePinch();
        sleep(250);
        moveArm(-0.8, 0.8);
        frontArm.setPower(-0.5);
        turnLeft(driveSpeed, 33, 3);
        left(driveSpeed, 3, 2);
        liftUp(1.0, 1255, 4);
        forward(driveSpeed, 14, 3);
        sleep(150);
        moveArm(0.2, 0.1);
        sleep(150);
        openPinch();
        frontArm.setPower(-0.5);
        backward(driveSpeed, 5, 1);
        liftDown(1.0, 1340, 4);
        turnRight(driveSpeed, 29.5, 3);
        
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
            target = (int)(inches*COUNTS_PER_INCH*(globalCorection-0.05));
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
            
            while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) || (leftHexLift.isBusy() || rightHexLift.isBusy()) && (runtime.seconds() < timeout)) {
                
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
            target = (int)(inches*COUNTS_PER_INCH*(globalCorection-0.05));
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
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) && (leftHexLift.isBusy() && rightHexLift.isBusy())) {
                
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
    
    public void forward(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*(globalCorection-0.05));
            
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void backward(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*globalCorection);
            
            leftFrontDrive.setTargetPosition(-target);
            rightFrontDrive.setTargetPosition(-target);
            leftBackDrive.setTargetPosition(-target);
            rightBackDrive.setTargetPosition(-target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void right(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*globalCorection);
            
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(-target);
            leftBackDrive.setTargetPosition(-target);
            rightBackDrive.setTargetPosition(target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void left(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*globalCorection);
            
            leftFrontDrive.setTargetPosition(-target);
            rightFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(-target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void turnRight(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*globalCorection);
            
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(-target);
            leftBackDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(-target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
    }
    
    public void turnLeft(double speed, double inches, int timeout){
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int target;
        
        if(opModeIsActive()){
            
            target = (int)(inches*COUNTS_PER_INCH*globalCorection);
            
            leftFrontDrive.setTargetPosition(-target);
            rightFrontDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(-target);
            rightBackDrive.setTargetPosition(target);
            
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            
            while (opModeIsActive() && (runtime.seconds() < timeout) && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Front-at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Path2",  "Back--at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            
            //Stop Drive Motors
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            
            sleep(globalSleep);
            
        }
        
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
    
    //Send The Calulated Power from calcPower Function.
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
    private double waitWhile(double time){
        
        runtime.reset();
        
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", " %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
        return runtime.seconds();
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
    
    
    /*Timed Drive Functions*/
    private void driveAxial(double power, double time){
        axial = power;
        sendPower();
        waitWhile(time);
        stopDrive();
        
    }
    
    private void driveLateral(double power, double time){
        lateral = power;
        sendPower();
        waitWhile(time);
        stopDrive();
        
    }
    
    private void lift(double power, double time){
        liftPower = power;
        sendPower();
        waitWhile(time);
        liftPower = 0;
        sendPower();
        
    }
    
    private void correctYaw(double power, double time){
        yaw = power;
        sendPower();
        waitWhile(time);
        yaw = 0.0;
        sendPower();
        stopDrive();
        
    }
    
    private void moveArm(double power, double time){
        frontArm.setPower(power);
        sendPower();
        waitWhile(time);
        frontArm.setPower(0.0);
    }
}



