package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * AutoHelperQuickTest - Quick test for AutoHelper functionality
 * 
 * This is a simple autonomous OpMode that demonstrates basic AutoHelper
 * functionality and can be used to quickly test that everything is working.
 * 
 * @author FTC Team
 */
@Autonomous(name="AutoHelper Quick Test", group="Testing")
public class AutoHelperQuickTest extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing AutoHelper...");
        telemetry.update();
        
        // Initialize AutoHelper with basic settings
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.setDebugTelemetryEnabled(true);
        autoHelper.setDefaultParameters(0.5, 3000); // Slower, shorter timeouts for testing
        
        try {
            // Try dual camera first, fall back to single, then encoders only
            autoHelper.initialize("Webcam 1", "Webcam 2");
            telemetry.addData("Camera", "Dual camera initialized");
        } catch (Exception e) {
            try {
                autoHelper.initialize("Webcam 1");
                telemetry.addData("Camera", "Single camera initialized");
            } catch (Exception e2) {
                autoHelper.initialize(null, null);
                telemetry.addData("Camera", "Encoders only");
            }
        }
        
        telemetry.addData("Status", "Ready - Press Start");
        telemetry.addData("Test", "Will run basic movement test");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            runQuickTest();
        }
    }
    
    private void runQuickTest() {
        telemetry.addData("Test", "Starting quick movement test");
        telemetry.update();
        
        // Simple test sequence
        boolean success = autoHelper
            .moveBy(6, 0, 0, "Move forward 6 inches")
            .waitFor(500, "Brief pause")
            .turnBy(90, "Turn right 90 degrees")
            .waitFor(500, "Brief pause")
            .moveBy(-6, 0, 0, "Move back 6 inches")
            .turnBy(-90, "Turn back to start")
            .executeAll();
        
        // Report results
        telemetry.addData("Test Result", success ? "SUCCESS" : "FAILED");
        
        if (!success) {
            telemetry.addData("Error", autoHelper.getLastErrorMessage());
        }
        
        // Show final position if available
        if (autoHelper.aph != null) {
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f, H: %.1f°",
                autoHelper.aph.getCurrentX(),
                autoHelper.aph.getCurrentY(), 
                autoHelper.aph.getCurrentHeading());
        }
        
        telemetry.addData("Steps", autoHelper.getCurrentStepInfo());
        telemetry.addData("Status", "Test complete");
        telemetry.update();
        
        // Keep results visible
        sleep(10000);
    }
}
