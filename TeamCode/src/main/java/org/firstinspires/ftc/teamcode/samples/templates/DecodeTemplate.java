/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * Template for DECODE season autonomous with shooting
 * Copy this file and modify for your specific autonomous routine
 */

package org.firstinspires.ftc.teamcode.samples.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.AutoHelper;
import org.firstinspires.ftc.teamcode.util.DecodeHelper;

/**
 * Template for DECODE autonomous with integrated shooting
 * 
 * This template shows the most common patterns for using DecodeHelper with AAF:
 * - Basic shooting sequences
 * - Movement with shooting
 * - Shooter management
 */
@Autonomous(name="DECODE Template", group="Templates")
public class DecodeTemplate extends LinearOpMode {
    
    private AutoHelper autoHelper;
    private DecodeHelper decodeHelper;
    
    @Override
    public void runOpMode() {
        // Initialize both helpers
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        decodeHelper = new DecodeHelper(hardwareMap, telemetry);
        
        // Configure AutoHelper for your robot
        autoHelper.initialize("Webcam 1", "Webcam 2");  // Change camera names as needed
        autoHelper.setRelocalizationEnabled(true);      // Use AprilTag correction
        autoHelper.setStuckDetectionEnabled(true);      // Handle getting stuck
        autoHelper.setDefaultParameters(0.6, 5000);     // 60% power, 5s timeout
        
        telemetry.addData("Status", "Ready to start autonomous");
        telemetry.update();
        
        waitForStart();
        
        // =================================================================
        // AUTONOMOUS SEQUENCE - Modify this section for your strategy
        // =================================================================
        
        // Example: Move to shooting position and fire artifacts
        autoHelper
            .moveTo(24, 36, 90, "Move to basket shooting position")
            .addStep("Start shooter", decodeHelper.createStartShooterAction())
            .waitFor(1200, "Wait for shooter to spin up")
            .addStep("Fire 3 artifacts", decodeHelper.createShootAction(3, false))
            .executeAll();
        
        // Example: Move and shoot in one sequence
        autoHelper.clearSteps();
        decodeHelper.addStartShooterStep(autoHelper, "Start shooter for sequence")
            .waitFor(1000, "Shooter spinup")
            .moveTo(48, 24, 45, "Move to second position");
        
        decodeHelper.addShootingSequence(autoHelper, 2, "Fire 2 shots at second position");
        
        autoHelper.executeAll();
        
        // Example: Pick up specimens and shoot
        autoHelper.clearSteps();
        autoHelper
            .moveTo(12, 60, 0, "Move to specimen area")
            .addStep("Grab specimen", () -> {
                // Add your specimen grabbing code here
                // For example: grabberServo.setPosition(0.5);
                sleep(500);
                return true;
            })
            .moveTo(24, 36, 90, "Move back to shooting position");
        
        decodeHelper.addShootingSequence(autoHelper, 1, "Fire specimen");
        
        autoHelper.executeAll();
        
        // =================================================================
        // END OF AUTONOMOUS SEQUENCE
        // =================================================================
        
        // Final status and cleanup
        telemetry.addData("Autonomous", "Complete");
        telemetry.addData("Status", autoHelper.isExecutionComplete() ? "SUCCESS" : "PARTIAL");
        decodeHelper.updateTelemetry();
        telemetry.update();
        
        // Keep robot stopped and telemetry visible
        sleep(1000);
    }
}

/*
 * USAGE NOTES FOR DECODE HELPER WITH AAF:
 * 
 * 1. SIMPLE SHOOTING:
 *    decodeHelper.addShootingSequence(autoHelper, numShots, "description");
 * 
 * 2. SHOOTER CONTROL:
 *    decodeHelper.addStartShooterStep(autoHelper, "description");
 *    decodeHelper.addStopShooterStep(autoHelper, "description");
 *    
 * 3. CUSTOM ACTIONS:
 *    autoHelper.addStep("description", decodeHelper.createShootAction(shots, keepRunning));
 *    
 * 4. NON-BLOCKING SHOOTING (advanced):
 *    java.util.function.Supplier<Boolean> shootAction = 
 *        decodeHelper.createNonBlockingShootAction(shots, keepRunning);
 *    autoHelper.addStep("description", shootAction);
 * 
 * 5. MIXED WITH MOVEMENT:
 *    autoHelper
 *        .moveTo(x, y, heading, "description")
 *        .addStep("shoot", decodeHelper.createShootAction(shots, keepRunning))
 *        .moveTo(x2, y2, heading2, "next position");
 * 
 * FIELD COORDINATES (DECODE 2025):
 * - Origin (0,0) is field center
 * - +X is toward audience (front of field)  
 * - +Y is from Red Wall toward Blue Alliance
 * - Heading 0° faces toward Blue Alliance (+Y)
 * 
 * COMMON POSITIONS:
 * - Basket area: (24, 36) to (36, 48)
 * - Specimen pickup: (12, 60) area
 * - Starting positions: Check your alliance and starting tile
 */