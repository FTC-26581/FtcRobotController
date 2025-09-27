/* Copyright (c) 2025 FTC Team. All rights reserved.
 *
 * DECODE Helper Class for artifact launching and collection functionality
 * This class provides methods for teleop button integration and autonomous operation
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AutoHelper;

/**
 * DecodeHelper - A utility class for DECODE season artifact launching
 * 
 * Features:
 * - Single shot launching for teleop button presses
 * - Continuous shooting when button held down
 * - Auto-compatible methods for autonomous operations
 * - State management for proper timing and sequencing
 */
public class DecodeHelper {
    
    // Hardware components
    private DcMotor shooter;
    private CRServo feedServo1;
    private CRServo feedServo2;
    
    // Timing and state management
    private ElapsedTime timer;
    private Telemetry telemetry;
    
    // Configuration constants
    private static final double SHOOTER_POWER = 0.75;
    private static final double FEED_POWER = 1.0;
    private static final double FEED_TIME = 0.3; // Time to run feed servos for one shot
    private static final double SHOT_INTERVAL = 1.5; // Minimum time between shots
    private static final double SHOOTER_SPINUP_TIME = 2.0; // Time for shooter to reach speed
    
    // State variables
    private boolean shooterRunning = false;
    private boolean isShooting = false;
    private double lastShotTime = 0;
    private boolean prevButtonState = false;
    
    /**
     * Constructor - Initialize the DecodeHelper
     * @param hardwareMap The robot's hardware map
     * @param telemetry Telemetry for debugging output
     */
    public DecodeHelper(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        feedServo1 = hardwareMap.get(CRServo.class, "servo1");
        feedServo2 = hardwareMap.get(CRServo.class, "servo2");
        
        // Configure shooter motor
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize timing and telemetry
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        
        // Reset state
        reset();
    }
    
    /**
     * Reset all systems to initial state
     */
    public void reset() {
        stopShooter();
        stopFeedServos();
        isShooting = false;
        lastShotTime = 0;
        prevButtonState = false;
        timer.reset();
    }
    
    /**
     * Start the shooter motor
     */
    public void startShooter() {
        shooter.setPower(SHOOTER_POWER);
        shooterRunning = true;
    }
    
    /**
     * Stop the shooter motor
     */
    public void stopShooter() {
        shooter.setPower(0.0);
        shooterRunning = false;
    }
    
    /**
     * Start the feed servos to launch an artifact
     */
    private void startFeedServos() {
        feedServo1.setPower(-FEED_POWER);
        feedServo2.setPower(FEED_POWER);
    }
    
    /**
     * Stop the feed servos
     */
    private void stopFeedServos() {
        feedServo1.setPower(0.0);
        feedServo2.setPower(0.0);
    }
    
    /**
     * Check if shooter is at speed and ready to fire
     * @return true if shooter has been running long enough to be at speed
     */
    public boolean isShooterReady() {
        return shooterRunning && (timer.seconds() >= SHOOTER_SPINUP_TIME) && (timer.seconds() - lastShotTime >= SHOT_INTERVAL);
    }
    
    /**
     * Fire a single shot (non-blocking)
     * Call this method repeatedly in your main loop
     * @return true if a shot was fired this cycle
     */
    public boolean fireSingleShot() {
        double currentTime = timer.seconds();
        
        if (!isShooting && isShooterReady() && (currentTime - lastShotTime >= SHOT_INTERVAL)) {
            // Start feeding
            startFeedServos();
            //telemetry.addData("stopped feed", !isShooting && isShooterReady() && (currentTime - lastShotTime >= SHOT_INTERVAL));
            isShooting = true;
            lastShotTime = currentTime;
            return true;
        }
        
        // Check if we should stop feeding
        if (isShooting && (currentTime - lastShotTime >= FEED_TIME)) {
            stopFeedServos();
            //telemetry.addData("stopped feed", isShooting && (currentTime - lastShotTime >= FEED_TIME));
            isShooting = false;
        }
        
        return false;
    }
    
    /**
     * Handle teleop button press for shooting
     * This method manages button state and provides single-shot or continuous shooting
     * Call this in your teleop loop with the current button state
     * 
     * @param buttonPressed Current state of the shoot button
     * @return true if a shot was initiated this cycle
     */
    public boolean handleShootButton(boolean buttonPressed) {
        boolean shotFired = false;

        // Edge: button pressed -> start shooter & spinup timer
        if (buttonPressed && !prevButtonState) {
            if (!shooterRunning) {
                startShooter();
                timer.reset();
            }
        }

        // Edge: button released -> stop shooter AND make sure feeds are stopped
        if (!buttonPressed && prevButtonState) {
            stopShooter();
            // safety: guarantee feeds stop if we let go during a shot
            if (isShooting) {
                stopFeedServos();
                isShooting = false;
            }
        }

        // Always service an in-flight shot so it can stop after FEED_TIME
        if (isShooting) {
            fireSingleShot(); // this will stop the servos when time elapses
        } else if (buttonPressed && isShooterReady()) {
            // Only initiate a new shot when ready
            shotFired = fireSingleShot(); // will start the feed and set isShooting
        }

        prevButtonState = buttonPressed;
        return shotFired;
    }
    
    /**
     * Autonomous shooting method - fires a specified number of shots
     * This is a blocking method suitable for autonomous routines
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after shots complete
     */
    public void autoShoot(int numShots, boolean keepShooterRunning) {
        if (numShots <= 0) return;
        
        // Start shooter if not already running
        if (!shooterRunning) {
            startShooter();
        }
        
        // Wait for shooter to spin up
        ElapsedTime spinupTimer = new ElapsedTime();
        while (spinupTimer.seconds() < SHOOTER_SPINUP_TIME) {
            telemetry.addData("Shooter", "Spinning up... %4.1f", spinupTimer.seconds());
            telemetry.update();
        }
        
        // Fire the requested number of shots
        for (int i = 0; i < numShots; i++) {
            telemetry.addData("Shooting", "Shot %d of %d", i + 1, numShots);
            telemetry.update();
            
            // Feed artifact
            startFeedServos();
            
            // Wait for feed time
            ElapsedTime feedTimer = new ElapsedTime();
            while (feedTimer.seconds() < FEED_TIME) {
                // Keep updating telemetry during feed
                telemetry.addData("Feeding artifact", "%4.1f seconds", feedTimer.seconds());
                telemetry.update();
            }
            
            // Stop feeding
            stopFeedServos();
            
            // Wait between shots (except for last shot)
            if (i < numShots - 1) {
                ElapsedTime intervalTimer = new ElapsedTime();
                while (intervalTimer.seconds() < SHOT_INTERVAL) {
                    telemetry.addData("Shot interval", "%4.1f seconds", intervalTimer.seconds());
                    telemetry.update();
                }
            }
        }
        
        // Stop shooter if requested
        if (!keepShooterRunning) {
            stopShooter();
        }
        
        telemetry.addData("Auto Shoot", "Complete - %d shots fired", numShots);
        telemetry.update();
    }
    
    /**
     * Manual shooter control for fine-tuning
     * @param power Power level (0.0 to 1.0)
     */
    public void setShooterPower(double power) {
        shooter.setPower(power);
        shooterRunning = (power > 0);
    }
    
    /**
     * Manual feed servo control
     * @param power Power level (-1.0 to 1.0)
     */
    public void setFeedPower(double power) {
        feedServo1.setPower(power);
        feedServo2.setPower(-power);
    }
    
    /**
     * Get current shooter power
     * @return Current shooter motor power
     */
    public double getShooterPower() {
        return shooter.getPower();
    }
    
    /**
     * Check if shooter is currently running
     * @return true if shooter is running
     */
    public boolean isShooterRunning() {
        return shooterRunning;
    }
    
    /**
     * Check if currently in the process of shooting
     * @return true if feed servos are currently running
     */
    public boolean isShooting() {
        return isShooting;
    }
    
    /**
     * Get time since last shot
     * @return seconds since last shot was fired
     */
    public double getTimeSinceLastShot() {
        if(lastShotTime>timer.seconds()){
            return 0;//to prevent negative return.
        }else{
            return timer.seconds() - lastShotTime;
        }
    }
    
    /**
     * Update telemetry with current status
     * Call this in your main loop to see DecodeHelper status
     */
    public void updateTelemetry() {
        telemetry.addData("Shooter Power", "%.2f", getShooterPower());
        telemetry.addData("Shooter Ready", isShooterReady() ? "Yes" : "No");
        telemetry.addData("Currently Shooting", isShooting() ? "Yes" : "No");
        telemetry.addData("Time Since Last Shot", "%.1f sec", getTimeSinceLastShot());
    }
    
    // ========== AAF (AUTONOMOUS ACTION FRAMEWORK) INTEGRATION ==========
    
    /**
     * Creates a Supplier<Boolean> for shooting that's compatible with AutoHelper.addStep()
     * This allows DecodeHelper shooting to be integrated into AAF autonomous sequences
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after completion
     * @return Supplier that returns true when shooting is complete
     */
    public java.util.function.Supplier<Boolean> createShootAction(int numShots, boolean keepShooterRunning) {
        return () -> {
            autoShoot(numShots, keepShooterRunning);
            return true; // autoShoot is blocking, so always returns true when complete
        };
    }
    
    /**
     * Creates a non-blocking shooting action for AAF that can be called repeatedly
     * Use this for more advanced autonomous routines where you need to do other things while shooting
     * 
     * @param numShots Number of shots to fire
     * @param keepShooterRunning Whether to keep shooter running after completion
     * @return Supplier that manages shooting state and returns true when complete
     */
    public java.util.function.Supplier<Boolean> createNonBlockingShootAction(int numShots, boolean keepShooterRunning) {
        return new java.util.function.Supplier<Boolean>() {
            private int shotsFired = 0;
            private boolean initialized = false;
            private double lastShotStartTime = 0;
            private boolean currentlyShooting = false;
            
            @Override
            public Boolean get() {
                double currentTime = timer.seconds();
                
                // Initialize on first call
                if (!initialized) {
                    if (!shooterRunning) {
                        startShooter();
                    }
                    initialized = true;
                    lastShotStartTime = currentTime - SHOOTER_SPINUP_TIME; // Allow immediate first shot
                    return false; // Not complete yet
                }
                
                // Check if we're done
                if (shotsFired >= numShots) {
                    if (!keepShooterRunning) {
                        stopShooter();
                    }
                    return true; // Complete
                }
                
                // Handle shooting timing
                if (!currentlyShooting && isShooterReady() && 
                    (currentTime - lastShotStartTime >= SHOT_INTERVAL)) {
                    // Start next shot
                    startFeedServos();
                    currentlyShooting = true;
                    lastShotStartTime = currentTime;
                } else if (currentlyShooting && 
                          (currentTime - lastShotStartTime >= FEED_TIME)) {
                    // Stop current shot
                    stopFeedServos();
                    currentlyShooting = false;
                    shotsFired++;
                    
                    // Update telemetry
                    telemetry.addData("Non-blocking Shot", "%d of %d complete", shotsFired, numShots);
                    telemetry.update();
                }
                
                return false; // Not complete yet
            }
        };
    }
    
    /**
     * Create a simple shooter start action for AAF
     * @return Supplier that starts the shooter and returns true
     */
    public java.util.function.Supplier<Boolean> createStartShooterAction() {
        return () -> {
            startShooter();
            return true;
        };
    }
    
    /**
     * Create a simple shooter stop action for AAF
     * @return Supplier that stops the shooter and returns true
     */
    public java.util.function.Supplier<Boolean> createStopShooterAction() {
        return () -> {
            stopShooter();
            return true;
        };
    }
    
    /**
     * Create a wait-for-shooter-ready action for AAF
     * @return Supplier that returns true when shooter is ready to fire
     */
    public java.util.function.Supplier<Boolean> createWaitForShooterReadyAction() {
        return () -> isShooterReady();
    }
    
    /**
     * Create an action that fires a single shot (non-blocking)
     * Call this repeatedly until it returns true
     * @return Supplier that manages single shot and returns true when complete
     */
    public java.util.function.Supplier<Boolean> createSingleShotAction() {
        return new java.util.function.Supplier<Boolean>() {
            private boolean shotInitiated = false;
            private double shotStartTime = 0;
            
            @Override
            public Boolean get() {
                double currentTime = timer.seconds();
                
                if (!shotInitiated && isShooterReady()) {
                    // Start the shot
                    startFeedServos();
                    shotInitiated = true;
                    shotStartTime = currentTime;
                    lastShotTime = currentTime;
                    return false; // Not complete yet
                } else if (shotInitiated && (currentTime - shotStartTime >= FEED_TIME)) {
                    // Complete the shot
                    stopFeedServos();
                    return true; // Complete
                }
                
                return shotInitiated ? false : isShooterReady(); // Wait for shooter ready if not initiated
            }
        };
    }
    
    // ========== AAF UTILITY METHODS ==========
    
    /**
     * Helper method to add shooting steps to an AutoHelper instance
     * This provides a convenient way to add common shooting patterns
     * 
     * Usage:
     * DecodeHelper decodeHelper = new DecodeHelper(hardwareMap, telemetry);
     * decodeHelper.addShootingSequence(autoHelper, 3, "Fire 3 artifacts at basket");
     * 
     * @param autoHelper The AutoHelper instance to add steps to
     * @param numShots Number of shots to fire
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addShootingSequence(
            AutoHelper autoHelper, 
            int numShots, 
            String description) {
        return autoHelper.addStep(description, createShootAction(numShots, false));
    }
    
    /**
     * Helper method to add a shooter start step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addStartShooterStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createStartShooterAction());
    }
    
    /**
     * Helper method to add a shooter stop step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addStopShooterStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createStopShooterAction());
    }
    
    /**
     * Helper method to add a wait-for-shooter-ready step to AutoHelper
     * @param autoHelper The AutoHelper instance to add steps to
     * @param description Description for telemetry
     * @return The same AutoHelper instance for chaining
     */
    public AutoHelper addWaitForShooterReadyStep(
            AutoHelper autoHelper, 
            String description) {
        return autoHelper.addStep(description, createWaitForShooterReadyAction());
    }
}