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

//import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BasicDrive25", group="Linear OpMode")

public class BasicDrive25 extends LinearOpMode {

    // Define constants at the top of your class
    private static final double SLOW_DRIVE_SCALE = 1.0 / 3.0;
    private static final long BUTTON_DEBOUNCE_MS = 150;

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

    //Variables for arm
    double armCPR = 288;

    //Pincher Finger Position
    double pinchPos = 0.0;
    double pinch2Pos = 0.0;

    public void mechanum(){

        if((gamepad1.left_stick_y!=0)||(gamepad1.right_stick_x!=0)||(gamepad1.right_trigger!=0)||(gamepad1.left_trigger!=0)){
            // POV Mode uses left joystick to go forward, and right joystick to rotate, and triggers to strafe.
            axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  -(gamepad1.right_trigger-gamepad1.left_trigger);
            yaw     =  -gamepad1.right_stick_x;

            if(slowDrive==1){
                axial *= SLOW_DRIVE_SCALE;
                lateral *= SLOW_DRIVE_SCALE;
                yaw *= SLOW_DRIVE_SCALE;
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

    public void toggleSlowDrive() {
        slowDrive = (slowDrive == 0) ? 1 : 0;
        // Optional: Add telemetry feedback if needed.
        telemetry.addData("Slow Drive", slowDrive == 1 ? "Enabled" : "Disabled");
        telemetry.update();
    }

    boolean prevSlowToggle = false;

    // Declare OpMode members for DC motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontArm = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //DC Motor Mapping and Power Behavior
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Drive Train Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until STOP is pressed
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button && !prevSlowToggle) {
                toggleSlowDrive();
            }
            prevSlowToggle = gamepad1.left_stick_button;

            mechanum();

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //TELEMETRY DATA CODE
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
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


    /*Time Functions*/
    //Function to wait specified time in seconds without stopping robot.
    private void waitWhile(double time){

        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", " %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }
}

