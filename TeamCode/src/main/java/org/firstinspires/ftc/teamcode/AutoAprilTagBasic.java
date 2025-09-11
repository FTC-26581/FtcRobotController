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

        // Example: Look for a tag for up to 5 seconds, then drive forward if found
        long startTime = System.currentTimeMillis();
        boolean tagFound = false;
        int targetTagId = 1; // Change to your desired tag ID
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 5000 && !tagFound) {
            List<AprilTagDetection> detections = aprilTagUtil.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    tagFound = true;
                    telemetry.addData("Tag Found!", detection.id);
                    break;
                }
            }
            aprilTagUtil.addTelemetry(this);
            telemetry.update();
            sleep(50);
        }

        if (tagFound) {
            // Drive forward for 1 second
            drive.drive(0.5, 0, 0); // Forward
            sleep(1000);
            drive.drive(0, 0, 0); // Stop
        } else {
            telemetry.addData("Tag Not Found", "No action taken");
            telemetry.update();
        }

        aprilTagUtil.close();
    }
}
