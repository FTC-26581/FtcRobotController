package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MechanumFieldRelative;
import org.firstinspires.ftc.teamcode.util.AprilTagMultiTool;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="DriveWithAprilTag", group="Linear OpMode")
public class DriveWithAprilTag extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean prevSlowToggle = false;
    private boolean prevModeToggle = false;
    private boolean fieldRelativeMode = false;
    private MechanumFieldRelative drive;
    private AprilTagMultiTool aprilTagUtil;

    @Override
    public void runOpMode() {
        // Map motors
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Mechanum drive
        drive = new MechanumFieldRelative(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, gamepad1, hardwareMap);

        // AprilTag utility (single webcam, change name if needed)
        aprilTagUtil = new AprilTagMultiTool(hardwareMap, true, "Webcam 1", null);
        aprilTagUtil.resumeStreaming();

        telemetry.addData("Status", "Robot & AprilTag Initialized");
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
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                drive.dpadMove();
            } else {
                if (fieldRelativeMode) {
                    drive.driveFieldRelative(forward, strafe, rotate);
                } else {
                    drive.drive(forward, strafe, rotate);
                }
            }

            // AprilTag telemetry
            aprilTagUtil.addTelemetry(this);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Mode", fieldRelativeMode ? "Field Relative" : "Robot Centric");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFrontPower, drive.rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBackPower, drive.rightBackPower);
            telemetry.update();
        }

        aprilTagUtil.close();
    }
}
