package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AutoHelper - Comprehensive Autonomous Programming Helper Class
 * 
 * This class provides all the essential functionality needed for autonomous OpModes:
 * - Hardware initialization and management
 * - Step-based autonomous execution
 * - Movement functions (time-based and encoder-based)
 * - Integration with Mechanum drive classes
 * - Telemetry and debugging utilities
 * - Safety and error handling
 * 
 * Usage:
 * 1. Create an instance in your autonomous OpMode
 * 2. Call initialize() in your runOpMode() method
 * 3. Use the movement and step functions to build your autonomous routine
 * 
 * @author FTC Team
 * @version 1.0
 */
public class AutoHelper {
    
    // Hardware objects
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    
    // Drive systems
    private MechanumDrive robotDrive = null;
    private MechanumFieldRelative fieldRelativeDrive = null;
    private AdvancedPositioningHelper positioningHelper = null;
    
    // OpMode references
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    
    // Step control
    private int currentStep = 0;
    private boolean stepComplete = false;
    private ElapsedTime stepTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Movement parameters
    public static final double DEFAULT_DRIVE_SPEED = 0.6;
    public static final double DEFAULT_TURN_SPEED = 0.4;
    public static final double DEFAULT_STRAFE_SPEED = 0.5;
    
    // Encoder constants (adjust based on your robot)
    public static final double COUNTS_PER_MOTOR_REV = 537.7;    // HD Hex Motor
    public static final double DRIVE_GEAR_REDUCTION = 1.0;      // No External Gearing
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // Wheel diameter
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double ROBOT_WIDTH_INCHES = 18.0;       // Distance between left and right wheels
    public static final double DEGREES_TO_INCHES = (ROBOT_WIDTH_INCHES * 3.1415) / 360.0;
    
    // Motor name configuration
    public static final String LEFT_FRONT_MOTOR = "frontLeft";
    public static final String RIGHT_FRONT_MOTOR = "frontRight";
    public static final String LEFT_BACK_MOTOR = "backLeft";
    public static final String RIGHT_BACK_MOTOR = "backRight";
    
    // Drive mode enumeration
    public enum DriveMode {
        BASIC_MECHANUM,    // Use MechanumDrive class
        FIELD_RELATIVE,    // Use MechanumFieldRelative class
        ADVANCED_POSITIONING // Use AdvancedPositioningHelper with sensor fusion
    }
    
    /**
     * Constructor
     * @param opMode The LinearOpMode instance
     */
    public AutoHelper(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }
    
    /**
     * Initialize all hardware and systems
     * @param driveMode Which drive system to use
     */
    public void initialize(DriveMode driveMode) {
        initializeMotors();
        initializeDriveSystem(driveMode);
        resetRuntime();
        
        telemetry.addData("AutoHelper", "Initialized with %s drive", driveMode.toString());
        telemetry.update();
    }
    
    /**
     * Initialize with advanced positioning system
     * @param driveMode Drive system to use
     * @param webcamName Name of webcam for AprilTag detection
     */
    public void initialize(DriveMode driveMode, String webcamName) {
        initializeMotors();
        initializeDriveSystem(driveMode);
        
        if (driveMode == DriveMode.ADVANCED_POSITIONING) {
            positioningHelper = new AdvancedPositioningHelper(opMode);
            positioningHelper.initialize(webcamName);
        }
        
        resetRuntime();
        
        telemetry.addData("AutoHelper", "Initialized with %s drive", driveMode.toString());
        if (webcamName != null) {
            telemetry.addData("Camera", webcamName);
        }
        telemetry.update();
    }
    
    /**
     * Initialize with basic mechanum drive (default)
     */
    public void initialize() {
        initialize(DriveMode.BASIC_MECHANUM);
    }
    
    // ========================================
    // HARDWARE INITIALIZATION
    // ========================================
    
    /**
     * Initialize drive motors
     */
    private void initializeMotors() {
        // Map motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        rightFrontDrive = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);
        leftBackDrive = hardwareMap.get(DcMotor.class, LEFT_BACK_MOTOR);
        rightBackDrive = hardwareMap.get(DcMotor.class, RIGHT_BACK_MOTOR);
        
        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders
        resetEncoders();
    }
    
    /**
     * Initialize drive system
     */
    private void initializeDriveSystem(DriveMode driveMode) {
        switch (driveMode) {
            case BASIC_MECHANUM:
                robotDrive = new MechanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, opMode.gamepad1);
                break;
            case FIELD_RELATIVE:
                fieldRelativeDrive = new MechanumFieldRelative(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, opMode.gamepad1, hardwareMap);
                break;
            case ADVANCED_POSITIONING:
                // AdvancedPositioningHelper is initialized separately with camera
                break;
        }
    }
    
    // ========================================
    // STEP MANAGEMENT
    // ========================================
    
    /**
     * Move to the next step
     */
    public void nextStep() {
        currentStep++;
        stepComplete = false;
        stepTimer.reset();
    }
    
    /**
     * Go to a specific step
     */
    public void goToStep(int step) {
        currentStep = step;
        stepComplete = false;
        stepTimer.reset();
    }
    
    /**
     * Reset steps to beginning
     */
    public void resetSteps() {
        currentStep = 0;
        stepComplete = false;
        stepTimer.reset();
    }
    
    /**
     * Get current step number
     */
    public int getCurrentStep() {
        return currentStep;
    }
    
    /**
     * Check if current step is complete
     */
    public boolean isStepComplete() {
        return stepComplete;
    }
    
    /**
     * Get step timer
     */
    public double getStepTime() {
        return stepTimer.seconds();
    }
    
    // ========================================
    // TIME-BASED MOVEMENT FUNCTIONS
    // ========================================
    
    /**
     * Drive with specified powers for a duration
     * @param seconds Duration to drive
     * @param forward Forward power (-1.0 to 1.0)
     * @param strafe Strafe power (-1.0 to 1.0)
     * @param rotate Rotation power (-1.0 to 1.0)
     * @return true when movement is complete
     */
    public boolean driveFor(double seconds, double forward, double strafe, double rotate) {
        if (!stepComplete) {
            stepTimer.reset();
            stepComplete = true;
        }
        
        if (stepTimer.seconds() < seconds && opMode.opModeIsActive()) {
            // Calculate and set motor powers
            setMechanumPowers(forward, strafe, rotate);
            return false;
        } else {
            stopAllMotors();
            nextStep();
            return true;
        }
    }
    
    /**
     * Drive forward for specified time
     */
    public boolean driveForward(double seconds, double speed) {
        return driveFor(seconds, speed, 0, 0);
    }
    
    /**
     * Drive backward for specified time
     */
    public boolean driveBackward(double seconds, double speed) {
        return driveFor(seconds, -speed, 0, 0);
    }
    
    /**
     * Strafe left for specified time
     */
    public boolean strafeLeft(double seconds, double speed) {
        return driveFor(seconds, 0, speed, 0);
    }
    
    /**
     * Strafe right for specified time
     */
    public boolean strafeRight(double seconds, double speed) {
        return driveFor(seconds, 0, -speed, 0);
    }
    
    /**
     * Turn left (counterclockwise) for specified time
     */
    public boolean turnLeft(double seconds, double speed) {
        return driveFor(seconds, 0, 0, speed);
    }
    
    /**
     * Turn right (clockwise) for specified time
     */
    public boolean turnRight(double seconds, double speed) {
        return driveFor(seconds, 0, 0, -speed);
    }
    
    // ========================================
    // ENCODER-BASED MOVEMENT FUNCTIONS
    // ========================================
    
    /**
     * Drive a specific distance using encoders
     * @param forwardInches Forward distance (positive = forward)
     * @param strafeInches Strafe distance (positive = left)
     * @param rotationDegrees Rotation (positive = counterclockwise)
     * @param speed Motor speed (0.0 to 1.0)
     * @return true when movement is complete
     */
    public boolean driveDistance(double forwardInches, double strafeInches, double rotationDegrees, double speed) {
        if (!stepComplete) {
            // Calculate target positions
            int forwardTarget = (int)(forwardInches * COUNTS_PER_INCH);
            int strafeTarget = (int)(strafeInches * COUNTS_PER_INCH);
            int rotationTarget = (int)(rotationDegrees * DEGREES_TO_INCHES * COUNTS_PER_INCH);
            
            // Calculate individual motor targets
            int leftFrontTarget = leftFrontDrive.getCurrentPosition() + forwardTarget + strafeTarget + rotationTarget;
            int rightFrontTarget = rightFrontDrive.getCurrentPosition() + forwardTarget - strafeTarget - rotationTarget;
            int leftBackTarget = leftBackDrive.getCurrentPosition() + forwardTarget - strafeTarget + rotationTarget;
            int rightBackTarget = rightBackDrive.getCurrentPosition() + forwardTarget + strafeTarget - rotationTarget;
            
            // Set target positions
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);
            
            // Set to RUN_TO_POSITION mode
            setRunToPositionMode();
            
            // Set motor powers
            setMotorPowers(speed, speed, speed, speed);
            
            stepComplete = true;
        }
        
        // Check if movement is complete
        if (!isMoving()) {
            stopAllMotors();
            setRunUsingEncoderMode();
            nextStep();
            return true;
        }
        
        return false;
    }
    
    /**
     * Drive forward a specific distance
     */
    public boolean driveForwardDistance(double inches, double speed) {
        return driveDistance(inches, 0, 0, speed);
    }
    
    /**
     * Drive backward a specific distance
     */
    public boolean driveBackwardDistance(double inches, double speed) {
        return driveDistance(-inches, 0, 0, speed);
    }
    
    /**
     * Strafe left a specific distance
     */
    public boolean strafeLeftDistance(double inches, double speed) {
        return driveDistance(0, inches, 0, speed);
    }
    
    /**
     * Strafe right a specific distance
     */
    public boolean strafeRightDistance(double inches, double speed) {
        return driveDistance(0, -inches, 0, speed);
    }
    
    /**
     * Turn left a specific number of degrees
     */
    public boolean turnLeftDegrees(double degrees, double speed) {
        return driveDistance(0, 0, degrees, speed);
    }
    
    /**
     * Turn right a specific number of degrees
     */
    public boolean turnRightDegrees(double degrees, double speed) {
        return driveDistance(0, 0, -degrees, speed);
    }
    
    // ========================================
    // UTILITY FUNCTIONS
    // ========================================
    
    /**
     * Wait for a specified amount of time
     * @param seconds Time to wait
     * @return true when wait is complete
     */
    public boolean waitFor(double seconds) {
        if (!stepComplete) {
            stepTimer.reset();
            stepComplete = true;
        }
        
        if (stepTimer.seconds() >= seconds) {
            nextStep();
            return true;
        }
        
        return false;
    }
    
    /**
     * Set mechanum drive powers with normalization
     */
    private void setMechanumPowers(double forward, double strafe, double rotate) {
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;
        
        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));
        
        setMotorPowers(leftFrontPower / max, rightFrontPower / max, 
                      leftBackPower / max, rightBackPower / max);
    }
    
    /**
     * Set individual motor powers
     */
    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }
    
    /**
     * Stop all motors
     */
    public void stopAllMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
    
    /**
     * Check if any motor is still moving
     */
    public boolean isMoving() {
        return leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || 
               leftBackDrive.isBusy() || rightBackDrive.isBusy();
    }
    
    /**
     * Reset all encoders (FTC best practice)
     * Always call this at the beginning of OpModes to ensure encoders start at zero
     */
    public void resetEncoders() {
        // Stop and reset all motor encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Turn motors back on for use (required after STOP_AND_RESET_ENCODER)
        setRunWithoutEncoderMode();
        
        telemetry.addData("Encoders", "Reset to zero");
        telemetry.update();
    }
    
    /**
     * Set motors to RUN_WITHOUT_ENCODER mode (FTC best practice for autonomous)
     * This doesn't disable encoders, it just disables built-in velocity control
     */
    private void setRunWithoutEncoderMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * Set motors to RUN_USING_ENCODER mode (for velocity control)
     */
    private void setRunUsingEncoderMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Set motors to RUN_TO_POSITION mode
     */
    private void setRunToPositionMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    /**
     * Reset runtime timer
     */
    public void resetRuntime() {
        runtime.reset();
    }
    
    /**
     * Get total runtime
     */
    public double getRuntime() {
        return runtime.seconds();
    }
    
    // ========================================
    // ADVANCED POSITIONING FUNCTIONS
    // ========================================
    
    /**
     * Move robot to exact field position using advanced positioning
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @param targetHeading Target heading (degrees)
     * @param maxSpeed Maximum movement speed
     * @return true when position is reached
     */
    public boolean goToPosition(double targetX, double targetY, double targetHeading, double maxSpeed) {
        if (positioningHelper == null) {
            telemetry.addData("Error", "Advanced positioning not initialized");
            telemetry.update();
            return true;
        }
        
        if (!stepComplete) {
            stepTimer.reset();
            stepComplete = true;
        }
        
        if (positioningHelper.goToPosition(targetX, targetY, targetHeading, maxSpeed)) {
            nextStep();
            return true;
        }
        
        return false;
    }
    
    /**
     * Move robot to exact field position (no heading change)
     * @param targetX Target X coordinate (inches)
     * @param targetY Target Y coordinate (inches)
     * @param maxSpeed Maximum movement speed
     * @return true when position is reached
     */
    public boolean goToPosition(double targetX, double targetY, double maxSpeed) {
        if (positioningHelper == null) {
            telemetry.addData("Error", "Advanced positioning not initialized");
            telemetry.update();
            return true;
        }
        
        return goToPosition(targetX, targetY, positioningHelper.getCurrentHeading(), maxSpeed);
    }
    
    /**
     * Rotate robot to exact heading using advanced positioning
     * @param targetHeading Target heading (degrees)
     * @param maxSpeed Maximum turn speed
     * @return true when heading is reached
     */
    public boolean rotateToHeading(double targetHeading, double maxSpeed) {
        if (positioningHelper == null) {
            telemetry.addData("Error", "Advanced positioning not initialized");
            telemetry.update();
            return true;
        }
        
        if (!stepComplete) {
            stepTimer.reset();
            stepComplete = true;
        }
        
        if (positioningHelper.rotateToHeading(targetHeading, maxSpeed)) {
            nextStep();
            return true;
        }
        
        return false;
    }
    
    /**
     * Set robot's current position (for calibration)
     * @param x Current X position (inches)
     * @param y Current Y position (inches)
     * @param heading Current heading (degrees)
     */
    public void setCurrentPosition(double x, double y, double heading) {
        if (positioningHelper != null) {
            positioningHelper.resetPosition(x, y, heading);
        }
    }
    
    /**
     * Get robot's current X position
     * @return X position in inches
     */
    public double getCurrentX() {
        return positioningHelper != null ? positioningHelper.getCurrentX() : 0.0;
    }
    
    /**
     * Get robot's current Y position
     * @return Y position in inches
     */
    public double getCurrentY() {
        return positioningHelper != null ? positioningHelper.getCurrentY() : 0.0;
    }
    
    /**
     * Get robot's current heading
     * @return Heading in degrees
     */
    public double getCurrentHeading() {
        return positioningHelper != null ? positioningHelper.getCurrentHeading() : 0.0;
    }
    
    /**
     * Get distance to target position
     * @param targetX Target X coordinate
     * @param targetY Target Y coordinate
     * @return Distance in inches
     */
    public double getDistanceToTarget(double targetX, double targetY) {
        return positioningHelper != null ? positioningHelper.getDistanceToTarget(targetX, targetY) : 0.0;
    }
    
    /**
     * Check if AprilTag positioning is available
     * @return true if AprilTag fix is available
     */
    public boolean hasAprilTagFix() {
        return positioningHelper != null && positioningHelper.hasAprilTagFix();
    }
    
    /**
     * Calibrate IMU with known heading
     * @param knownHeading The robot's actual heading (degrees)
     */
    public void calibrateIMU(double knownHeading) {
        if (positioningHelper != null) {
            positioningHelper.calibrateIMU(knownHeading);
        }
    }
    
    /**
     * Update positioning system (call this regularly when using advanced positioning)
     */
    public void updatePosition() {
        if (positioningHelper != null) {
            positioningHelper.updatePosition();
        }
    }
    
    // ========================================
    // TELEMETRY FUNCTIONS
    // ========================================
    
    /**
     * Update telemetry with current status
     */
    public void updateTelemetry() {
        telemetry.addData("AutoHelper Status", "Running");
        telemetry.addData("Current Step", currentStep);
        telemetry.addData("Step Complete", stepComplete);
        telemetry.addData("Step Time", "%.2f", stepTimer.seconds());
        telemetry.addData("Total Runtime", "%.2f", runtime.seconds());
        
        // Position data if available
        if (positioningHelper != null) {
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", 
                    getCurrentX(), getCurrentY(), getCurrentHeading());
            telemetry.addData("AprilTag Fix", hasAprilTagFix() ? "YES" : "NO");
        }
        
        // Motor positions
        telemetry.addData("Encoders", "LF:%d RF:%d LB:%d RB:%d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        
        // Motor powers
        telemetry.addData("Powers", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f",
                leftFrontDrive.getPower(),
                rightFrontDrive.getPower(),
                leftBackDrive.getPower(),
                rightBackDrive.getPower());
        
        // Update positioning system telemetry if available
        if (positioningHelper != null) {
            positioningHelper.updateTelemetry();
        } else {
            telemetry.update();
        }
    }
    
    /**
     * Add custom telemetry data
     */
    public void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }
    
    /**
     * Add custom telemetry data
     */
    public void addTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
    }
    
    // ========================================
    // ADVANCED FUNCTIONS
    // ========================================
    
    /**
     * Drive using field-relative control (requires field relative drive system)
     * @param seconds Duration to drive
     * @param forward Forward power in field coordinates
     * @param strafe Strafe power in field coordinates
     * @param rotate Rotation power
     * @return true when movement is complete
     */
    public boolean driveFieldRelativeFor(double seconds, double forward, double strafe, double rotate) {
        if (fieldRelativeDrive == null) {
            telemetry.addData("Error", "Field relative drive not initialized");
            telemetry.update();
            return true;
        }
        
        if (!stepComplete) {
            stepTimer.reset();
            stepComplete = true;
        }
        
        if (stepTimer.seconds() < seconds && opMode.opModeIsActive()) {
            fieldRelativeDrive.driveFieldRelative(forward, strafe, rotate);
            return false;
        } else {
            stopAllMotors();
            nextStep();
            return true;
        }
    }
    
    /**
     * Check if OpMode is still active (safety check)
     */
    public boolean isActive() {
        return opMode.opModeIsActive();
    }
    
    /**
     * Get motor by name for advanced users
     */
    public DcMotor getMotor(String motorName) {
        switch (motorName.toLowerCase()) {
            case "leftfront":
            case "left_front":
            case "frontleft":
                return leftFrontDrive;
            case "rightfront":
            case "right_front":
            case "frontright":
                return rightFrontDrive;
            case "leftback":
            case "left_back":
            case "backleft":
                return leftBackDrive;
            case "rightback":
            case "right_back":
            case "backright":
                return rightBackDrive;
            default:
                return null;
        }
    }
    
    /**
     * Get reference to the basic mechanum drive system
     */
    public MechanumDrive getRobotDrive() {
        return robotDrive;
    }
    
    /**
     * Get reference to the field relative drive system
     */
    public MechanumFieldRelative getFieldRelativeDrive() {
        return fieldRelativeDrive;
    }
    
    /**
     * Get reference to the advanced positioning system
     */
    public AdvancedPositioningHelper getAdvancedPositioning() {
        return positioningHelper;
    }
    
    // ========================================
    // ENCODER UTILITY METHODS (FTC BEST PRACTICES)
    // ========================================
    
    /**
     * Get encoder position in counts (ticks)
     * @param motor Which motor encoder to read
     * @return Position in encoder counts
     */
    public int getEncoderCounts(String motor) {
        switch (motor.toLowerCase()) {
            case "leftfront":
            case "frontleft":
                return leftFrontDrive.getCurrentPosition();
            case "rightfront":
            case "frontright":
                return rightFrontDrive.getCurrentPosition();
            case "leftback":
            case "backleft":
                return leftBackDrive.getCurrentPosition();
            case "rightback":
            case "backright":
                return rightBackDrive.getCurrentPosition();
            default:
                return 0;
        }
    }
    
    /**
     * Get encoder position in revolutions
     * @param motor Which motor encoder to read
     * @return Position in revolutions
     */
    public double getEncoderRevolutions(String motor) {
        int counts = getEncoderCounts(motor);
        return counts / COUNTS_PER_MOTOR_REV;
    }
    
    /**
     * Get encoder position in inches (linear distance)
     * @param motor Which motor encoder to read
     * @return Position in inches
     */
    public double getEncoderInches(String motor) {
        int counts = getEncoderCounts(motor);
        return counts / COUNTS_PER_INCH;
    }
    
    /**
     * Get encoder angle in degrees (0-360, normalized)
     * @param motor Which motor encoder to read
     * @return Angle in degrees (normalized to 0-360)
     */
    public double getEncoderAngle(String motor) {
        double revolutions = getEncoderRevolutions(motor);
        double angle = revolutions * 360.0;
        return angle % 360.0;
    }
    
    /**
     * Get encoder angle in degrees (total, can exceed 360)
     * @param motor Which motor encoder to read
     * @return Total angle in degrees
     */
    public double getEncoderAngleTotal(String motor) {
        double revolutions = getEncoderRevolutions(motor);
        return revolutions * 360.0;
    }
    
    /**
     * Get all encoder positions formatted for telemetry
     * @return Formatted string with all encoder data
     */
    public String getEncoderTelemetry() {
        return String.format("LF:%d RF:%d LB:%d RB:%d", 
            leftFrontDrive.getCurrentPosition(),
            rightFrontDrive.getCurrentPosition(), 
            leftBackDrive.getCurrentPosition(),
            rightBackDrive.getCurrentPosition());
    }
    
    /**
     * Get encoder positions in inches for telemetry
     * @return Formatted string with encoder distances
     */
    public String getEncoderDistances() {
        return String.format("LF:%.1f\" RF:%.1f\" LB:%.1f\" RB:%.1f\"",
            getEncoderInches("leftfront"),
            getEncoderInches("rightfront"),
            getEncoderInches("leftback"), 
            getEncoderInches("rightback"));
    }
    
    /**
     * Cleanup resources (call at end of OpMode)
     */
    public void close() {
        if (positioningHelper != null) {
            positioningHelper.close();
        }
    }
}