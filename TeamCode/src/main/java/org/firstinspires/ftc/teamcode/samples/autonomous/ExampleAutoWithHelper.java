package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoHelper;
import org.firstinspires.ftc.teamcode.util.AdvancedPositioningHelper;

/**
 * ExampleAutoWithHelper - Demonstrates the new AutoHelper Framework
 * 
 * This OpMode shows how to use the AutoHelper class to create simple,
 * readable autonomous programs with intelligent error detection and recovery.
 * 
 * Key features demonstrated:
 * - Fluent API for step-by-step programming
 * - Automatic error detection and recovery
 * - AprilTag-based relocalization
 * - Comprehensive telemetry and debugging
 * 
 * @author FTC Team
 */
@Autonomous(name="Example Auto with Helper", group="Examples")
public class ExampleAutoWithHelper extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    @Override
    public void runOpMode() {
        // Initialize AutoHelper with dual camera support
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        
        // Configure advanced features
        autoHelper.setRelocalizationEnabled(true);    // Enable AprilTag recovery
        autoHelper.setStuckDetectionEnabled(true);    // Enable stuck detection
        autoHelper.setDebugTelemetryEnabled(true);    // Show detailed telemetry
        autoHelper.setDefaultParameters(0.7, 4000);   // 70% power, 4 second timeout
        
        // Initialize with dual cameras (front and back)
        autoHelper.initialize("Webcam 1", "Webcam 2");
        
        telemetry.addData("Status", "Initialized, waiting for start");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Example 1: Simple fluent API usage
            simpleFluentExample();
            
            // Wait between examples
            sleep(1000);
            
            // Example 2: Advanced step-based programming
            advancedStepExample();
        }
    }
    
    /**
     * Example 1: Simple Fluent API Usage
     * 
     * This demonstrates the easiest way to create autonomous routines.
     * Each method call adds a step that will be executed in sequence.
     */
    private void simpleFluentExample() {
        telemetry.addData("Example", "Starting Simple Fluent API Example");
        telemetry.update();
        
        autoHelper
            // Move to scoring position
            .moveTo(24, 36, 90, "Move to high basket")
            
            // Wait for arm to extend (simulated)
            .waitFor(1000, "Wait for arm extension")
            
            // Approach the basket
            .moveBy(4, 0, 0, "Approach basket")
            
            // Wait for scoring (simulated)
            .waitFor(500, "Score sample")
            
            // Back away from basket
            .moveBy(-8, 0, 0, "Back away from basket")
            
            // Turn to face specimen area
            .turnTo(0, "Turn to specimen area")
            
            // Move to specimen pickup
            .moveTo(12, 60, 0, "Move to specimen pickup")
            
            // Execute all steps with basic error handling
            .executeAll();
        
        // Display results
        telemetry.addData("Simple Example", "Completed: %s", 
            autoHelper.isExecutionComplete() ? "Success" : "Failed");
        if (autoHelper.hasExecutionFailed()) {
            telemetry.addData("Error", autoHelper.getLastErrorMessage());
        }
        telemetry.update();
    }
    
    /**
     * Example 2: Advanced Step-Based Programming
     * 
     * This demonstrates more advanced features including custom actions,
     * conditional steps, and recovery mechanisms.
     */
    private void advancedStepExample() {
        telemetry.addData("Example", "Starting Advanced Step Example");
        telemetry.update();
        
        // Clear any previous steps
        autoHelper.clearSteps();
        
        autoHelper
            // Custom step with complex logic
            .addStep("Initialize arm system", () -> {
                // Simulate arm initialization
                telemetry.addData("Action", "Initializing arm system");
                telemetry.update();
                sleep(500);
                
                // Return true for success, false for failure
                return true;
            })
            
            // Move with custom parameters (higher speed, longer timeout)
            .moveTo(48, 24, 45, 0.8, 6000, "Drive to submersible")
            
            // Wait until a condition is met (with timeout)
            .waitUntil(() -> {
                // Simulate checking if arm is in position
                // In real code, this would check sensor values
                return true;
            }, 3000, "Wait for arm to reach position")
            
            // Custom action that doesn't return a value
            .addAction("Grab specimen", () -> {
                telemetry.addData("Action", "Grabbing specimen");
                telemetry.update();
                sleep(1000);
            })
            
            // Turn by relative angle
            .turnBy(90, "Turn to face baskets")
            
            // Final movement
            .moveTo(24, 36, 135, "Move to low basket")
            
            // Execute with advanced recovery (AprilTag relocalization if stuck)
            .executeAllWithRecovery();
        
        // Display comprehensive results
        telemetry.addData("Advanced Example", "Completed: %s", 
            autoHelper.isExecutionComplete() ? "Success" : "Failed");
        
        if (autoHelper.hasExecutionFailed()) {
            telemetry.addData("Error", autoHelper.getLastErrorMessage());
        }
        
        // Show detailed step information
        telemetry.addData("Current Step", autoHelper.getCurrentStepInfo());
        
        // Display current robot position
        if (autoHelper.aph != null) {
            telemetry.addData("Final Position", "X: %.1f, Y: %.1f, H: %.1f°",
                autoHelper.aph.getCurrentX(),
                autoHelper.aph.getCurrentY(), 
                autoHelper.aph.getCurrentHeading());
        }
        
        telemetry.update();
        
        // Optional: Get detailed execution report for debugging
        String report = autoHelper.getExecutionReport();
        // You could log this to a file or display key parts
    }
    
    /**
     * Example 3: Direct APH Access
     * 
     * Sometimes you need direct access to the AdvancedPositioningHelper
     * for more complex operations.
     */
    private void directAphExample() {
        // You can always access the APH directly
        AdvancedPositioningHelper aph = autoHelper.getPositioningHelper();
        
        // Use APH methods directly when needed
        double currentX = aph.getCurrentX();
        double currentY = aph.getCurrentY();
        double currentHeading = aph.getCurrentHeading();
        
        // Or mix direct APH calls with AutoHelper steps
        autoHelper
            .addStep("Custom movement with direct APH", () -> {
                // Direct APH call with custom logic
                return aph.moveTo(currentX + 12, currentY - 6, currentHeading + 45, 0.5, 3000);
            })
            .executeAll();
    }
}