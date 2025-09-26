package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoHelper;

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
        
        // Set default movement traversal mode (optional)
        // DIRECT_VECTOR: Straight line to target (fastest, default)
        // X_THEN_Y: Move X first, then Y, then rotate (good for avoiding obstacles)
        // Y_THEN_X: Move Y first, then X, then rotate (alternative obstacle avoidance)
        // MANHATTAN_AUTO: Automatically choose X or Y first based on larger distance
        // SEPARATE_PHASES: Pure X, then pure Y, then rotate (most predictable)
        autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.DIRECT_VECTOR);
        
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
            // Example: Move to starting position (default traversal mode)
            .moveTo(12, 12, 0, "Move to starting position")
            
            // Example: Wait for something
            .waitFor(500, "Brief pause")
            
            // Example: Score preload using X-then-Y movement (avoids obstacles)
            .moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.X_THEN_Y, "Move to high basket via X-then-Y")
            .waitFor(1000, "Score preload")
            .moveBy(-6, 0, 0, "Back away")
            
            // Example: Go to specimen area using Y-then-X to avoid field elements
            .turnTo(0, "Turn to specimen area")
            .moveTo(12, 60, 0, AutoHelper.MovementTraversalMode.Y_THEN_X, "Move to specimen pickup")
            
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

/*
MOVEMENT TRAVERSAL MODES - When to Use Each:

1. DIRECT_VECTOR (Default)
   - Fastest movement to target
   - Good for open field areas
   - Robot moves in straight line to target
   - Use when: No obstacles in path, want maximum speed

2. X_THEN_Y  
   - Moves X coordinate first, then Y, then rotates
   - Good for avoiding obstacles in specific patterns
   - Use when: Need to go around obstacles, want predictable path
   - Example: Need to move right first to avoid submersible, then forward

3. Y_THEN_X
   - Moves Y coordinate first, then X, then rotates
   - Alternative obstacle avoidance pattern
   - Use when: Need to move forward/back first, then sideways
   - Example: Need to move forward to clear wall, then strafe to position

4. MANHATTAN_AUTO
   - Automatically chooses X or Y first based on larger distance
   - Good general-purpose obstacle avoidance
   - Use when: Want L-shaped movement but don't know which axis is larger
   - Example: General movement where you want to minimize total distance

5. SEPARATE_PHASES
   - Pure strafe, then pure forward/back, then pure rotation
   - Most predictable and debuggable movement
   - Each axis moves independently
   - Use when: Need precise control, debugging complex movements
   - Example: Fine positioning near scoring areas

USAGE EXAMPLES:

// Use specific traversal mode for one movement:
autoHelper.moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.X_THEN_Y, "Avoid obstacle");

// Set default mode for all movements:
autoHelper.setDefaultTraversalMode(AutoHelper.MovementTraversalMode.MANHATTAN_AUTO);

// Mix modes as needed:
autoHelper
    .moveTo(12, 12, 0, AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Quick move to staging")
    .moveTo(24, 36, 90, AutoHelper.MovementTraversalMode.X_THEN_Y, "Careful move to basket")
    .moveTo(0, 0, 0, AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Quick return home")
    .executeAll();
*/