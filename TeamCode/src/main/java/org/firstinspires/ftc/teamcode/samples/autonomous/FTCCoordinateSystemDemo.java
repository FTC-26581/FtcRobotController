package org.firstinspires.ftc.teamcode.samples.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoHelper;
import org.firstinspires.ftc.teamcode.util.AdvancedPositioningHelper;

/**
 * Demonstration OpMode showcasing the FTC Field Coordinate System
 * 
 * This OpMode demonstrates:
 * 1. Proper FTC coordinate system usage
 * 2. Movement to specific field positions
 * 3. Heading management using FTC standards
 * 4. Alliance-specific positioning
 * 
 * FTC Field Coordinate System Summary:
 * - Origin (0,0) at field center
 * - X-axis: Blue Alliance Wall (+X) to Red Alliance Wall (-X)
 * - Y-axis: Audience Side (+Y) to Away from Audience (-Y)  
 * - Heading: 0° faces positive Y (away from audience), increases clockwise
 * - Field dimensions: 144" x 144" (12' x 12')
 */
@Autonomous(name = "FTC Coordinate System Demo", group = "Demo")
public class FTCCoordinateSystemDemo extends LinearOpMode {

    private AutoHelper autoHelper;
    private boolean allianceIsRed = true; // Set based on alliance selection
    
    // Example field positions using FTC coordinate system
    private static final double[][] RED_ALLIANCE_POSITIONS = {
        {-60.0, -60.0, 0.0},   // Near Red Wall, away from audience
        {-60.0, 0.0, 90.0},    // Red Wall, field center, face Blue Alliance
        {-60.0, 60.0, 180.0},  // Red Wall, audience side, face audience
        {0.0, 0.0, 0.0}        // Field center, face away from audience
    };
    
    private static final double[][] BLUE_ALLIANCE_POSITIONS = {
        {60.0, -60.0, 0.0},    // Near Blue Wall, away from audience  
        {60.0, 0.0, 270.0},    // Blue Wall, field center, face Red Alliance
        {60.0, 60.0, 180.0},   // Blue Wall, audience side, face audience
        {0.0, 0.0, 0.0}        // Field center, face away from audience
    };
    
    @Override
    public void runOpMode() {
        // Initialize AutoHelper
        autoHelper = new AutoHelper(this, hardwareMap, telemetry);
        autoHelper.initialize("Webcam 1");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Coordinate System", "FTC Official");
        telemetry.addData("Alliance", allianceIsRed ? "Red" : "Blue");
        telemetry.addData("Instructions", "Press start to run coordinate demo");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Execute coordinate system demonstration
            demonstrateCoordinateSystem();
        }
    }
    
    private void demonstrateCoordinateSystem() {
        double[][] positions = allianceIsRed ? RED_ALLIANCE_POSITIONS : BLUE_ALLIANCE_POSITIONS;

        // Execute movement sequence using AutoHelper's fluent API
        AutoHelper sequence = autoHelper;

        for (int i = 0; i < positions.length; i++) {
            sequence = sequence.moveTo(positions[i][0], positions[i][1], positions[i][2],
                String.format("Move to position %d", i + 1))
                .waitFor(2000, String.format("Wait at position %d", i + 1));
        }
        
        // Execute all movements
        sequence.executeAll();

        telemetry.addData("Demo", "Complete!");
        telemetry.update();
    }
}
