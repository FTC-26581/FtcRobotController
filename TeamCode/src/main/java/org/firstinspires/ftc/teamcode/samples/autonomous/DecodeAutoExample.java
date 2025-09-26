/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Example Autonomous OpMode demonstrating DecodeHelper integration with AAF
 */

package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;
import org.firstinspires.ftc.teamcode.util.DecodeHelper;

/**
 * Example autonomous OpMode showing how to integrate DecodeHelper with AAF
 * 
 * This demonstrates multiple ways to use DecodeHelper with the AutoHelper framework:
 * 1. Using the helper methods for common patterns
 * 2. Using addStep() with custom DecodeHelper actions
 * 3. Non-blocking shooting while moving
 */
@Autonomous(name="DECODE Auto with Shooting", group="Examples")
public class DecodeAutoExample extends LinearOpMode {
    
    private AutoHelper autoHelper;
    private DecodeHelper decodeHelper;
    
    @Override
    public void runOpMode() {
        // Initialize both helpers
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        decodeHelper = new DecodeHelper(hardwareMap, telemetry);
        
        // Configure AutoHelper
        autoHelper.initialize("Webcam 1", "Webcam 2");  // Dual cameras
        autoHelper.setRelocalizationEnabled(true);
        autoHelper.setStuckDetectionEnabled(true);
        autoHelper.setDefaultParameters(0.6, 5000);  // 60% power, 5s timeout
        
        telemetry.addData("Status", "Initialized - Ready to start");
        telemetry.update();
        
        waitForStart();
        
        // Example 1: Simple shooting sequence using helper methods
        autoHelper
            .moveTo(24, 36, 90, "Move to first shooting position")
            .addStep("Start shooter and wait", decodeHelper.createStartShooterAction())
            .waitFor(1000, "Wait for shooter spinup")
            .addStep("Fire 3 shots", decodeHelper.createShootAction(3, true))
            .moveTo(48, 36, 90, "Move to second position")
            .addStep("Fire 2 more shots", decodeHelper.createShootAction(2, false))
            .executeAll();
        
        // Check if first sequence completed successfully
        if (!autoHelper.isExecutionComplete()) {
            telemetry.addData("Warning", "First sequence failed, continuing with backup");
            telemetry.update();
        }
        
        // Example 2: More advanced sequence with custom actions
        autoHelper.clearSteps(); // Clear previous steps
        
        autoHelper
            .moveTo(12, 60, 0, "Move to specimen pickup")
            .addStep("Custom specimen grab", () -> {
                // Your specimen grabbing code here
                sleep(500);
                telemetry.addData("Custom Action", "Specimen grabbed");
                telemetry.update();
                return true;
            })
            .moveTo(24, 36, 90, "Move to basket")
            .addStep("Score specimen", () -> {
                // Your specimen scoring code here
                sleep(300);
                telemetry.addData("Custom Action", "Specimen scored");
                telemetry.update();
                return true;
            })
            .executeAll();
        
        // Example 3: Non-blocking shooting while doing other things
        autoHelper.clearSteps();
        
        // Start a 5-shot sequence that runs in background
        java.util.function.Supplier<Boolean> nonBlockingShoot = 
            decodeHelper.createNonBlockingShootAction(5, false);
        
        autoHelper
            .addStep("Start non-blocking shooting", () -> {
                // This will initialize the shooting but not block
                nonBlockingShoot.get();
                return true;
            })
            .moveTo(36, 24, 45, "Move while shooting")
            .addStep("Continue shooting while moving", nonBlockingShoot)
            .moveTo(48, 48, 0, "Move to final position")
            .addStep("Finish shooting sequence", nonBlockingShoot)
            .executeAll();
        
        // Example 4: Using the convenient helper methods
        autoHelper.clearSteps();
        
        // This shows the most convenient way to add shooting to your autonomous
        decodeHelper.addStartShooterStep(autoHelper, "Prep shooter for final sequence")
            .waitFor(1000, "Shooter spinup time")
            .moveTo(60, 60, 180, "Move to corner");
        
        decodeHelper.addShootingSequence(autoHelper, 1, "Fire final shot");
        
        decodeHelper.addStopShooterStep(autoHelper, "Shutdown shooter");
        
        autoHelper.executeAll();
        
        // Final status
        telemetry.addData("Autonomous", "Complete");
        telemetry.addData("Final Status", autoHelper.isExecutionComplete() ? "SUCCESS" : "PARTIAL");
        telemetry.update();
        
        // Keep telemetry visible
        sleep(2000);
    }
}