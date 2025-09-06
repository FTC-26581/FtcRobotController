package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MechanumDrive {
    private static final double SLOW_DRIVE_SCALE = 1.0 / 3.0;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private Gamepad gamepad;
    private int slowDrive = 0;

    public double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    private double axial, lateral, yaw;
    private double max;

    public MechanumDrive(DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive, Gamepad gamepad) {
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightBackDrive = rightBackDrive;
        this.gamepad = gamepad;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void toggleSlowDrive() {
        slowDrive = (slowDrive == 0) ? 1 : 0;
    }

    public int getSlowDrive() {
        return slowDrive;
    }

    public void mechanum() {
        if ((gamepad.left_stick_y != 0) || (gamepad.right_stick_x != 0) || (gamepad.right_trigger != 0) || (gamepad.left_trigger != 0)) {
            axial = gamepad.left_stick_y;
            lateral = -(gamepad.right_trigger - gamepad.left_trigger);
            yaw = -gamepad.right_stick_x;
            if (slowDrive == 1) {
                axial *= SLOW_DRIVE_SCALE;
                lateral *= SLOW_DRIVE_SCALE;
                yaw *= SLOW_DRIVE_SCALE;
            }
        } else {
            dpadMove();
            yaw = 0;
        }
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
    }

    public void dpadMove() {
        if (gamepad.dpad_up) {
            axial = -0.2;
        } else if (gamepad.dpad_down) {
            axial = 0.2;
        } else {
            axial = 0;
        }
        if (gamepad.dpad_right) {
            lateral = -0.2;
        } else if (gamepad.dpad_left) {
            lateral = 0.2;
        } else {
            lateral = 0;
        }
    }

    public void setMotorPowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
