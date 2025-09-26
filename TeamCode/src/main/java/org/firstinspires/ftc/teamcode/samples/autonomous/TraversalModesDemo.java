package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * TraversalModesDemo - Demonstrates different movement traversal modes
 * 
 * This OpMode shows how the AutoHelper can move to the same target position
 * using different traversal strategies:
 * 
 * 1. DIRECT_VECTOR - Straight line (default APH behavior)
 * 2. X_THEN_Y - Move X first, then Y, then rotate
 * 3. Y_THEN_X - Move Y first, then X, then rotate  
 * 4. MANHATTAN_AUTO - Choose X or Y first based on larger distance
 * 5. SEPARATE_PHASES - X, Y, and rotation as completely separate phases
 * 
 * @author FTC Team
 */
@Autonomous(name="Traversal Modes Demo", group="Examples")
public class TraversalModesDemo extends LinearOpMode {
    
    private AutoHelper autoHelper;
    
    // Target positions to demonstrate (adjust for your field)
    private static final double TARGET_X = 24.0;
    private static final double TARGET_Y = 36.0;
    private static final double TARGET_HEADING = 90.0;
    
    @Override
    public void runOpMode() {
        // Initialize AutoHelper
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.setDebugTelemetryEnabled(true);
        autoHelper.setDefaultParameters(0.6, 4000);
        
        // Initialize with camera (adjust as needed)
        autoHelper.initialize("Webcam 1");
        
        telemetry.addData("Demo", "Traversal Modes Demonstration");
        telemetry.addData("Target", "X:%.1f Y:%.1f H:%.1f°", TARGET_X, TARGET_Y, TARGET_HEADING);
        telemetry.addData("", "Will demonstrate 5 different movement modes");
        telemetry.addData("Status", "Ready - Press Start");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            demonstrateTraversalModes();
        }
    }
    
    private void demonstrateTraversalModes() {
        // Demo 1: Direct Vector (Default behavior)
        telemetry.addData("Demo 1", "DIRECT_VECTOR - Straight line to target");
        telemetry.update();
        sleep(2000);
        
        autoHelper
            .clearSteps()
            .moveTo(TARGET_X, TARGET_Y, TARGET_HEADING, 
                AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Direct vector movement")
            .executeAll();
        
        showResults("Direct Vector");
        returnToStart();
        
        // Demo 2: X then Y
        telemetry.addData("Demo 2", "X_THEN_Y - Move X first, then Y, then rotate");
        telemetry.update();
        sleep(2000);
        
        autoHelper
            .clearSteps()
            .moveTo(TARGET_X, TARGET_Y, TARGET_HEADING, 
                AutoHelper.MovementTraversalMode.X_THEN_Y, "X then Y movement")
            .executeAll();
        
        showResults("X Then Y");
        returnToStart();
        
        // Demo 3: Y then X
        telemetry.addData("Demo 3", "Y_THEN_X - Move Y first, then X, then rotate");
        telemetry.update();
        sleep(2000);
        
        autoHelper
            .clearSteps()
            .moveTo(TARGET_X, TARGET_Y, TARGET_HEADING, 
                AutoHelper.MovementTraversalMode.Y_THEN_X, "Y then X movement")
            .executeAll();
        
        showResults("Y Then X");
        returnToStart();
        
        // Demo 4: Manhattan Auto
        telemetry.addData("Demo 4", "MANHATTAN_AUTO - Choose X or Y first automatically");
        telemetry.update();
        sleep(2000);
        
        autoHelper
            .clearSteps()
            .moveTo(TARGET_X, TARGET_Y, TARGET_HEADING, 
                AutoHelper.MovementTraversalMode.MANHATTAN_AUTO, "Manhattan auto movement")
            .executeAll();
        
        showResults("Manhattan Auto");
        returnToStart();
        
        // Demo 5: Separate Phases
        telemetry.addData("Demo 5", "SEPARATE_PHASES - Pure X, then pure Y, then rotate");
        telemetry.update();
        sleep(2000);
        
        autoHelper
            .clearSteps()
            .moveTo(TARGET_X, TARGET_Y, TARGET_HEADING, 
                AutoHelper.MovementTraversalMode.SEPARATE_PHASES, "Separate phases movement")
            .executeAll();
        
        showResults("Separate Phases");
        
        // Final summary
        telemetry.addData("Demo Complete", "All 5 traversal modes demonstrated");
        telemetry.addData("", "Check which mode works best for your robot/field");
        telemetry.update();
        sleep(5000);
    }
    
    private void showResults(String modeName) {
        telemetry.addData("Mode", modeName + " Complete");
        telemetry.addData("Target", "X:%.1f Y:%.1f H:%.1f°", TARGET_X, TARGET_Y, TARGET_HEADING);
        
        if (autoHelper.aph != null) {
            telemetry.addData("Actual", "X:%.1f Y:%.1f H:%.1f°",
                autoHelper.aph.getCurrentX(),
                autoHelper.aph.getCurrentY(), 
                autoHelper.aph.getCurrentHeading());
                
            // Calculate accuracy
            double xError = Math.abs(TARGET_X - autoHelper.aph.getCurrentX());
            double yError = Math.abs(TARGET_Y - autoHelper.aph.getCurrentY());
            double hError = Math.abs(TARGET_HEADING - autoHelper.aph.getCurrentHeading());
            if (hError > 180) hError = 360 - hError;
            
            telemetry.addData("Accuracy", "X:±%.1f\" Y:±%.1f\" H:±%.1f°", xError, yError, hError);
        }
        
        telemetry.addData("Status", autoHelper.isExecutionComplete() ? "SUCCESS" : "FAILED");
        if (autoHelper.hasExecutionFailed()) {
            telemetry.addData("Error", autoHelper.getLastErrorMessage());
        }
        
        telemetry.update();
        sleep(3000);
    }
    
    private void returnToStart() {
        telemetry.addData("Action", "Returning to start position...");
        telemetry.update();
        
        // Return to start with direct vector (fastest)
        autoHelper
            .clearSteps()
            .moveTo(0, 0, 0, AutoHelper.MovementTraversalMode.DIRECT_VECTOR, "Return to start")
            .executeAll();
        
        sleep(1000);
    }
}