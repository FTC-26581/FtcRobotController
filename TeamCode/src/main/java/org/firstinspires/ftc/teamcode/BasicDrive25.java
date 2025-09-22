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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MechanumFieldRelative;

@TeleOp(name="BasicDrive25", group="Linear OpMode")


public class BasicDrive25 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private boolean prevSlowToggle = false;
    private boolean prevModeToggle = false;
    private boolean fieldRelativeMode = false;
    private MechanumFieldRelative drive;
    private boolean shooterState = false;


    @Override
    public void runOpMode() {
        // Map motors with custom names if desired
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        CRServo servo1 = hardwareMap.get(CRServo.class, "servo1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "servo2");

        // Pass mapped motors to MechanumFieldRelative
        drive = new MechanumFieldRelative(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, gamepad1, hardwareMap);

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Toggle slow drive
            if (gamepad1.left_stick_button && !prevSlowToggle) {
                drive.toggleSlowDrive();
                telemetry.addData("Slow Drive", drive.getSlowDrive() == 1 ? "Enabled" : "Disabled");
                telemetry.update();
            }
            prevSlowToggle = gamepad1.left_stick_button;

            // Toggle field-relative/normal mode with X button
            if (gamepad1.x && !prevModeToggle) {
                fieldRelativeMode = !fieldRelativeMode;
                telemetry.addData("Drive Mode", fieldRelativeMode ? "Field Relative" : "Robot Centric");
                telemetry.update();
            }
            prevModeToggle = gamepad1.x;

            // Get joystick values
            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                // Use dpad for movement
                drive.dpadMove();
            } else {
                if (fieldRelativeMode) {
                    drive.driveFieldRelative(forward, strafe, rotate);
                } else {
                    drive.drive(forward, strafe, rotate);
                }
            }

            if(gamepad2.a){
                servo1.setPower(1.0);
                servo2.setPower(-1.0);
            } else {
                servo2.setPower(0.0);
                servo1.setPower(0.0);
            }

            if(gamepad2.triangle){
                shooterState = !shooterState;
                shooter.setPower(shooterState ? 1.0 : 0.0);
                waitWhile(0.5);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Mode", fieldRelativeMode ? "Field Relative" : "Robot Centric");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFrontPower, drive.rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBackPower, drive.rightBackPower);
            telemetry.update();
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

