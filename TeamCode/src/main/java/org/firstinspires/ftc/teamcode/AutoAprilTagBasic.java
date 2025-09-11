package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.AprilTagMultiTool;
import org.firstinspires.ftc.teamcode.MechanumFieldRelative;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name = "AutoAprilTagBasic", group = "Auto")
public class AutoAprilTagBasic extends LinearOpMode {
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


        // Path planning: Look for tags 1, 2, or 3 and drive a different path for each
        long startTime = System.currentTimeMillis();
        int detectedTagId = -1;
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 5000 && detectedTagId == -1) {
            List<AprilTagDetection> detections = aprilTagUtil.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
                    detectedTagId = detection.id;
                    telemetry.addData("Tag Found!", detectedTagId);
                    break;
                }
            }
            aprilTagUtil.addTelemetry(this);
            telemetry.update();
            sleep(50);
        }

        // Decision logic: choose path based on tag
        if (detectedTagId == 1) {
            telemetry.addData("Path", "Tag 1: Drive Forward");
            drive.drive(0.5, 0, 0); // Forward
            sleep(1000);
            drive.drive(0, 0, 0); // Stop
        } else if (detectedTagId == 2) {
            telemetry.addData("Path", "Tag 2: Strafe Left");
            drive.drive(0, -0.5, 0); // Left
            sleep(1000);
            drive.drive(0, 0, 0); // Stop
        } else if (detectedTagId == 3) {
            telemetry.addData("Path", "Tag 3: Strafe Right");
            drive.drive(0, 0.5, 0); // Right
            sleep(1000);
            drive.drive(0, 0, 0); // Stop
        } else {
            telemetry.addData("Tag Not Found", "No action taken");
        }
        telemetry.update();

        aprilTagUtil.close();
    }
}
