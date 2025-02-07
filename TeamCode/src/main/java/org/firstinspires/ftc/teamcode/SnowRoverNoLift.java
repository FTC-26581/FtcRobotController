/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Snow_Rover(NL/H) v1.7.5", group="Linear OpMode")

public class SnowRoverNoLift extends LinearOpMode {
    
    public void mechanum(){
        
        if((gamepad1.left_stick_y!=0)||(gamepad1.right_stick_x!=0)||(gamepad1.right_trigger!=0)||(gamepad1.left_trigger!=0)){
            // POV Mode uses left joystick to go forward, and right joystick to rotate, and tringers to strafe.
            axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  -(gamepad1.right_trigger-gamepad1.left_trigger);
            yaw     =  -gamepad1.right_stick_x;
            
            if(slowDrive==1){
                axial /= 3;
                lateral /= 3;
                yaw /= 3;
            }
            
        }else{
            dpadMove();
            yaw=0;
        }
        
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
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
    
    public void toggleSlowDrive(){
        if(slowDrive==0){
            slowDrive=1;
        }else{
            slowDrive=0;
        }
        sleep(150);
    }
    
    public void togglePincher(){
        if(pinchState==0){
            pinchState=1;
        }else{
            pinchState=0;
        }
    }
    
    

    // Declare OpMode members for DC motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightHexLift = null;
    private DcMotor leftHexLift = null;
    private DcMotor frontArm = null;
    //private DcMotor rightHang = null;
    //private DcMotor leftHang = null;
    
    //Define Servos//
    
    //Pincher servos
    Servo rightPinch;
    Servo leftPinch;
    
    //Mechanum Variables
    double max;
    
    double axial;
    double lateral;
    double yaw;
    
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    
    //Slow Drive State
    int slowDrive = 0;
    
    int pinchState = 1;
    
    //Variables for arm
    double armCPR = 288;
    
    //Pincher Finger Postion
    double pinchPos = 0.0;
    double pinch2Pos = 0.0;
    
    //Arm Servo Position
    double armPitch = 0.0;
    
    //Arm Motor Power
    double armPower;
    

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        
        //DC Motor Mapping and Power Behavior
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rightHexLift  = hardwareMap.get(DcMotor.class, "rightLift");
        leftHexLift  = hardwareMap.get(DcMotor.class, "leftLift");
        
        rightHexLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHexLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        frontArm  = hardwareMap.get(DcMotor.class, "frontArm");
        
        frontArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        //Servo Mapping
        rightPinch = hardwareMap.get(Servo.class, "rightPinch");
        leftPinch = hardwareMap.get(Servo.class, "leftPinch");

        //Drive Train Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        //Lift Motor Directions
        rightHexLift.setDirection(DcMotor.Direction.FORWARD);
        leftHexLift.setDirection(DcMotor.Direction.FORWARD);
    
    
        frontArm.setDirection(DcMotor.Direction.FORWARD);
        
        //Servo Directions
        rightPinch.setDirection(Servo.Direction.FORWARD);
        leftPinch.setDirection(Servo.Direction.REVERSE);
        
        
        //Variables for arm Encoder data
        int armPos = frontArm.getCurrentPosition();
        double armRevs = armPos/armCPR;
        
        double armAngle = armRevs * 360;
        double armNorm = armAngle % 360;
        
        
        

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until STOP is pressed
        while (opModeIsActive()) {
            
            double liftPower = (gamepad2.right_trigger-gamepad2.left_trigger);

            if(gamepad1.left_stick_button){
                toggleSlowDrive();
            }

            mechanum();

            //Pincher Servo Gamepad Control

            //Pinchers OPEN/CLOSE
            if(gamepad2.a){
                togglePincher();
                sleep(250);
            }
            
            //moves arm forward, opens pinchers and moves arm back
            if(gamepad2.b){
                moveArm(0.4, 0.2);
                sleep(150);
                openPinch();
                sleep(150);
                moveArm(-0.8, 0.3);
            }
            
            //moves arm down
            if(gamepad2.x){
                moveArm(0.3, 0.6);
            }
            
            //moves arm up
            if(gamepad2.y){
                moveArm(-0.8, 0.4);
                moveArm(-0.3, 0.2);
            }
            
            if(pinchState==1){
                closePinch();
            }else{
                openPinch();
            }
            

            if(gamepad2.left_bumper){
                armPower=0.3;
            }else if(gamepad2.right_bumper){
                armPower=-0.7;
            }else{
                armPower=0;
            }
            
            //Get Arm Encoder Data-----------------
            armPos = frontArm.getCurrentPosition();
            armRevs = armPos/armCPR;
            
            armAngle = armRevs * 360;
            armNorm = armAngle % 360;
            
            //frontArm.setTargetPosition(tgtArmPos);
            frontArm.setPower(armPower);
            
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            //Set Power for Lift Motors
            rightHexLift.setPower(liftPower);
            leftHexLift.setPower(liftPower);

            //Set Servo Positions and Power
            rightPinch.setPosition(pinch2Pos);
            leftPinch.setPosition(pinchPos);

            //TELEMETRY DATA CODE
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Position", armPitch);
            telemetry.addData("Pincher Position", pinchPos);
            //telemetry.addData("Arm Right Servo", rightArm.getPosition());
            //telemetry.addData("Arm Left Servo", leftArm.getPosition());
            telemetry.addData("Arm Power", frontArm.getPower());
            telemetry.addData("Arm Position", armPos);
            telemetry.addData("Arm Revolutions", armRevs);
            telemetry.addData("Arm Angle(degrees)", armAngle);
            telemetry.addData("Arm Normalized(degrees)", armNorm);
            //telemetry.addData("Arm Target", tgtArmPos);
            telemetry.update();
        }
    }
    
    public void openPinch(){
        pinchPos = 0.20;
        pinch2Pos = 0.35;
        
        rightPinch.setPosition(pinch2Pos);
        leftPinch.setPosition(pinchPos);
    }
    
    public void closePinch(){
        pinchPos = 0;
        pinch2Pos = 0;
        
        rightPinch.setPosition(pinch2Pos);
        leftPinch.setPosition(pinchPos);
    }
    
    public void dpadMove() {
        if(gamepad1.dpad_up){
            axial = -0.2;
        }else
        if(gamepad1.dpad_down) {
            axial = 0.2;
        }else{
            axial = 0;
        }
        
        
        if(gamepad1.dpad_right) {
            lateral = -0.2;
        }else
        if(gamepad1.dpad_left){
            lateral = 0.2;
        }else{
            lateral = 0;
        }
    }
    
    private int clipRange(int min, int max, int input){
        
        if(input>max){
            return max;
        }else if(input<min){
            return min;
        }else{
            return input;
        }
    }
    
    private double safeMove(double joystick, double safePos, double trigger, double pitch){
        if(joystick>trigger&&pitch>safePos){
            return safePos;
        }else{
            return pitch;
        }
    }
    
    private void moveArm(double power, double time){
        frontArm.setPower(power);
        waitWhile(time);
        frontArm.setPower(0.0);
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
    /*public void sendPower() {
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
    }*/
    
    //Set Power to Zero for all Motors
    /*private void stopDrive(){
        axial=0;
        lateral=0;
        yaw=0;
        liftPower = 0;
        sendPower();
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }*/
    
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
}

