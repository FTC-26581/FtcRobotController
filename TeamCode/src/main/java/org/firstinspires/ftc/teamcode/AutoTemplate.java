package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * AutoTemplate - Template for Creating Autonomous OpModes
 * 
 * Copy this file and modify it to create your own autonomous OpModes.
 * The AutoHelper framework makes it easy to create reliable autonomous programs.
 * 
 * Steps to use this template:
 * 1. Copy this file and rename it (e.g., "BlueLeftAuto.java")
 * 2. Change the class name to match the file name
 * 3. Update the @Autonomous annotation with your desired name
 * 4. Replace the example steps with your autonomous routine
 * 5. Test and iterate!
 * 
 * @author FTC Team
 */
@Autonomous(name="Auto Template", group="Templates")
public class AutoTemplate extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        // Initialize AutoHelper
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        
        // Configure features (customize as needed)
        autoHelper.setRelocalizationEnabled(true);    // Enable AprilTag recovery
        autoHelper.setStuckDetectionEnabled(true);    // Enable stuck detection
        autoHelper.setDebugTelemetryEnabled(true);    // Show detailed telemetry
        autoHelper.setDefaultParameters(0.6, 5000);   // 60% power, 5 second timeout
        
        // Initialize with cameras (adjust camera names as needed)
        // For single camera: autoHelper.initialize("Webcam 1");
        // For dual cameras: autoHelper.initialize("Webcam 1", "Webcam 2");
        // For encoders only: autoHelper.initialize(null, null);
        autoHelper.initialize("Webcam 1", "Webcam 2");
        
        telemetry.addData("Status", "Ready for autonomous");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Your autonomous routine goes here
            runAutonomousRoutine();
        }
    }
    
    /**
     * Define your autonomous routine here
     * 
     * Replace this example with your actual autonomous steps.
     * Use the fluent API for simple, readable code.
     */
    private void runAutonomousRoutine() {
        autoHelper
            // Example: Move to starting position
            .moveTo(12, 12, 0, "Move to starting position")
            
            // Example: Wait for something
            .waitFor(500, "Brief pause")
            
            // Example: Score preload
            .moveTo(24, 36, 90, "Move to high basket")
            .waitFor(1000, "Score preload")
            .moveBy(-6, 0, 0, "Back away")
            
            // Example: Go to specimen area
            .turnTo(0, "Turn to specimen area")
            .moveTo(12, 60, 0, "Move to specimen pickup")
            
            // Example: Custom action
            .addAction("Grab specimen", () -> {
                // Your custom code here
                // For example: activate claw, wait for sensors, etc.
                sleep(500);
            })
            
            // Example: Score specimen
            .moveTo(6, 36, 0, "Move to submersible")
            .addStep("Score specimen", () -> {
                // Your scoring logic here
                // Return true for success, false for failure
                return true;
            })
            
            // Example: Park
            .moveTo(48, 12, 0, "Park in observation zone")
            
            // Execute all steps with recovery
            .executeAllWithRecovery();
        
        // Display final results
        displayResults();
    }
    
    /**
     * Display final results and telemetry
     */
    private void displayResults() {
        telemetry.addData("Autonomous", "Completed: %s", 
            autoHelper.isExecutionComplete() ? "SUCCESS" : "FAILED");
        
        if (autoHelper.hasExecutionFailed()) {
            telemetry.addData("Error", autoHelper.getLastErrorMessage());
        }
        
        // Show final position
        if (autoHelper.aph != null) {
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f, H: %.1f°",
                autoHelper.aph.getCurrentX(),
                autoHelper.aph.getCurrentY(), 
                autoHelper.aph.getCurrentHeading());
        }
        
        telemetry.addData("Status", "Autonomous complete - check results above");
        telemetry.update();
        
        // Keep telemetry visible
        sleep(5000);
    }
    
    // ========== HELPER METHODS ==========
    // Add any custom helper methods you need here
    
    /**
     * Example: Custom scoring method
     */
    private boolean scoreInHighBasket() {
        // Your scoring logic here
        // This could involve arm movements, waiting for sensors, etc.
        
        telemetry.addData("Action", "Scoring in high basket");
        telemetry.update();
        sleep(1000); // Simulate scoring time
        
        return true; // Return true for success, false for failure
    }
    
    /**
     * Example: Custom specimen pickup method
     */
    private boolean pickupSpecimen() {
        // Your pickup logic here
        
        telemetry.addData("Action", "Picking up specimen");
        telemetry.update();
        sleep(800); // Simulate pickup time
        
        return true; // Return true for success, false for failure
    }
}