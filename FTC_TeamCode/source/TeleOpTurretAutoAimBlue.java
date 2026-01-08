package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TeleOp with Turret Auto-Aim for BLUE Alliance
 *
 * FEATURES:
 * - Turret auto-aims at goal using odometry
 * - Outtake uses PIDF velocity control for consistent shooting
 *
 * CONTROLS:
 * - LEFT BUMPER: Toggle turret auto-aim on/off
 * - RIGHT BUMPER: Quick increase outtake velocity (+200)
 * - RIGHT TRIGGER: Toggle intake on/off
 * - B: Toggle outtake on/off
 * - DPAD RIGHT: Increase outtake velocity (+50)
 * - DPAD LEFT: Decrease outtake velocity (-50)
 * - RIGHT STICK: Drive forward/backward and turn
 * - LEFT STICK: Strafe left/right
 */
@TeleOp(name = "TeleOp with Turret Auto-Aim - BLUE")
public class TeleOpTurretAutoAimBlue extends OpMode {

    // ==================== EXISTING HARDWARE ====================
    // Game element manipulation motors
    private DcMotor intake;          // Motor that spins to intake game elements
    private DcMotorEx outtake;       // First outtake motor for shooting (with encoder)
    private DcMotorEx outtake2;      // Second outtake motor for shooting (with encoder)

    // Drivetrain motors (4-wheel mecanum drive)
    private DcMotor FLeft;           // Front Left wheel
    private DcMotor BLeft;           // Back Left wheel
    private DcMotor Fright;          // Front Right wheel
    private DcMotor Bright;          // Back Right wheel

    // Servos for game element control
    // (No servos currently in use)

    // Utilities
    private ElapsedTime time;                      // Timer for time-based actions

    // ==================== TURRET HARDWARE ====================
    // Motor that rotates the turret to aim at the goal
    private DcMotorEx turretMotor;

    // Odometry computer that tracks robot position on the field
    private GoBildaPinpointDriver pinpoint;

    // ==================== TURRET CONSTANTS ====================
    // Location of the blue goal on the field (in inches)
    static final double BLUE_GOAL_X = 0;    // Goal is at X = 0
    static final double BLUE_GOAL_Y = 144;  // Goal is at Y = 144

    // Physical limits of turret rotation (±90 degrees)
    static final double MAX_TURRET_ANGLE = Math.toRadians(90);

    // Conversion factor: how many encoder ticks = 1 radian of rotation
    static final double TICKS_PER_RADIAN = 428;

    // Proportional control gain - how aggressively turret corrects errors
    // Higher = faster response but more oscillation
    static final double kP_TURRET = 0.5;

    // Maximum power sent to turret motor (0.0 to 1.0)
    static final double MAX_TURRET_POWER = 0.3;

    // ==================== ROBOT STARTING POSE ====================
    // Where the robot starts on the field (adjust for your starting position)
    double robotX = 56;                         // Starting X position in inches
    double robotY = 8;                          // Starting Y position in inches
    double robotHeading = Math.toRadians(90);   // Starting heading (90° = facing forward)

    // ==================== TURRET CONTROL ====================
    // Flag to track if turret auto-aim is currently active
    boolean turretAutoAimEnabled = false;

    // Used to detect button press (prevents multiple toggles from one press)
    private boolean lastLeftBumper = false;

    // ==================== EXISTING VARIABLES ====================
    // Button state tracking to detect single presses (not holds)
    private boolean lastTriggerPressed = false;
    private boolean lastOuttakeTogglePressed = false;

    // Toggle states for intake and outtake
    private boolean intakeOn = false;   // Is intake currently running?
    private boolean outtakeOn = false;  // Is outtake currently running?

    // Outtake velocity control (using PIDF for constant speed)
    // Based on measurements: 100% = 2200 ticks/sec, 75% = 1720, 50% = 1280
    // 58% calculated: 2200 * 0.58 = 1188 ticks/sec (matches ~50% measurement)
    private double targetVelocity = 1188;  // Target velocity in ticks per second (58% of max)
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    // PIDF Coefficients for velocity control - TUNED FOR GOBILDA 5203 MOTORS
    // System: Dual 96mm flywheels (284g), 879g total rotating mass, 0.165lb balls
    // Target: <3ms recovery time after ball contact (250-300 tick drop)
    private static final double kP_VELOCITY = 18.0;  // Aggressive proportional for fast response
    private static final double kI_VELOCITY = 3;   // Strong integral to eliminate steady-state error quickly
    private static final double kD_VELOCITY = 0.5;   // Small derivative to dampen oscillation
    private static final double kF_VELOCITY = 13.2;  // Feedforward calculated from velocity/power ratio (2200/100 * 0.6)

    @Override
    public void init() {
        time = new ElapsedTime();

        // ==================== EXISTING HARDWARE INIT ====================
        // Initialize all motors from hardware map
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        FLeft = hardwareMap.dcMotor.get("FL");
        BLeft = hardwareMap.dcMotor.get("BL");
        Fright = hardwareMap.dcMotor.get("FR");
        Bright = hardwareMap.dcMotor.get("BR");

        // ==================== TURRET HARDWARE INIT ====================
        // Initialize turret motor with encoder
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoder to 0
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);     // Run with power control
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  // Brake when power = 0

        // Initialize Pinpoint odometry computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set where the odometry pods are located on the robot (in millimeters)
        // IMPORTANT: Adjust these values to match YOUR robot's pod positions
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // Tell Pinpoint what type of odometry pods we're using
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Reset the odometry system
        pinpoint.resetPosAndIMU();

        // Set the robot's starting position on the field
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.RADIANS, robotHeading));

        // ==================== MOTOR DIRECTIONS ====================
        // Set motor directions so positive power moves robot forward/right
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ==================== OUTTAKE VELOCITY CONTROL SETUP ====================
        // Configure outtake motors for velocity control with PIDF
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control
        outtake.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);
        outtake2.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);

        telemetry.addData("Status", "Initialized - Ready for TeleOp");
        telemetry.addData("Turret Auto-Aim", "Press LEFT BUMPER to toggle");
        telemetry.update();
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        // ==================== UPDATE ODOMETRY ====================
        // CRITICAL: Must call this every loop to update robot position
        pinpoint.update();

        // Get the current robot position and heading from odometry
        Pose2D pos = pinpoint.getPosition();
        robotX = pos.getX(DistanceUnit.INCH);      // Current X position on field
        robotY = pos.getY(DistanceUnit.INCH);      // Current Y position on field
        robotHeading = pos.getHeading(AngleUnit.RADIANS);  // Current heading (rotation)

        // ==================== EXISTING CONTROLS ====================
        // Handle all the existing robot controls
        handleIntake();              // Right trigger toggles intake
        handleOuttakeToggle();       // B button toggles outtake
        handleOuttakeVelocityAdjust();  // D-pad left/right adjusts outtake velocity
        moveDriveTrain();            // Joysticks control robot movement

        // ==================== TURRET AUTO-AIM ====================
        // Handle turret aiming at the goal
        handleTurretAutoAim();

        // ==================== TELEMETRY ====================
        // Display information on driver station
        telemetry.addData("=== ROBOT INFO ===", "");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));

        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Auto-Aim", turretAutoAimEnabled ? "ON (L-Bumper to disable)" : "OFF (L-Bumper to enable)");
        telemetry.addData("Turret Encoder", turretMotor.getCurrentPosition());
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(turretMotor.getCurrentPosition() / TICKS_PER_RADIAN));

        telemetry.addData("=== INTAKE/OUTTAKE ===", "");
        telemetry.addData("Outtake On?", outtakeOn);
        telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Outtake1 Velocity", "%.0f ticks/sec", outtake.getVelocity());
        telemetry.addData("Outtake2 Velocity", "%.0f ticks/sec", outtake2.getVelocity());
        telemetry.addData("Outtake1 Power", "%.2f (%.0f%%)", outtake.getPower(), outtake.getPower() * 100);
        telemetry.addData("Outtake2 Power", "%.2f (%.0f%%)", outtake2.getPower(), outtake2.getPower() * 100);
        telemetry.addData("Velocity Error", "%.0f ticks/sec", targetVelocity - outtake.getVelocity());
        telemetry.addData("Intake On?", intakeOn);

        telemetry.update();
    }

    // ==================== TURRET AUTO-AIM HANDLER ====================
    /**
     * Handles toggling turret auto-aim on/off with the LEFT BUMPER
     * When enabled, turret automatically points at the blue goal
     */
    private void handleTurretAutoAim() {
        // Read current state of left bumper
        boolean leftBumper = gamepad1.left_bumper;

        // If button just pressed (wasn't pressed last loop, is pressed now)
        if (leftBumper && !lastLeftBumper) {
            turretAutoAimEnabled = !turretAutoAimEnabled;  // Toggle on/off
        }
        lastLeftBumper = leftBumper;  // Remember state for next loop

        // If auto-aim is enabled, aim turret at goal
        if (turretAutoAimEnabled) {
            aimTurretAtBlueGoal(robotX, robotY, robotHeading);
        } else {
            // If disabled, stop turret motor
            turretMotor.setPower(0);
        }
    }

    // ==================== TURRET AIM LOGIC ====================
    /**
     * Calculates and executes turret aiming at the blue goal
     * Uses proportional control to smoothly rotate turret to the correct angle
     *
     * @param x Current robot X position (inches)
     * @param y Current robot Y position (inches)
     * @param heading Current robot heading (radians)
     */
    private void aimTurretAtBlueGoal(double x, double y, double heading) {
        // STEP 1: Calculate the angle from robot to goal (in field coordinates)
        // atan2 gives us the angle in radians from the robot's position to the goal
        double angleToGoal = Math.atan2(
                BLUE_GOAL_Y - y,  // Vertical distance to goal
                BLUE_GOAL_X - x   // Horizontal distance to goal
        );

        // STEP 2: Convert field angle to robot-relative angle
        // Subtract robot heading to get angle relative to where robot is facing
        // normalizeRadians ensures angle is between -π and +π
        double desiredTurretAngle = AngleUnit.normalizeRadians(angleToGoal - heading);

        // STEP 3: Clamp to physical limits of turret (±90 degrees)
        // Turret can't rotate more than 90 degrees left or right
        desiredTurretAngle = Range.clip(
                desiredTurretAngle,
                -MAX_TURRET_ANGLE,  // -90 degrees
                MAX_TURRET_ANGLE    // +90 degrees
        );

        // STEP 4: Get current turret angle from encoder
        // Convert encoder ticks to radians using calibration constant
        double currentTurretAngle = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;

        // STEP 5: Calculate error (how far off we are)
        // Positive error = need to rotate counterclockwise
        // Negative error = need to rotate clockwise
        double error = AngleUnit.normalizeRadians(desiredTurretAngle - currentTurretAngle);

        // STEP 6: Apply proportional control
        // Multiply error by gain factor to get motor power
        // Larger error = more power (faster correction)
        double power = error * kP_TURRET;

        // STEP 7: Clip power to safe maximum
        // Prevent motor from getting too much power
        power = Range.clip(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        // STEP 8: Apply deadband to prevent jitter
        // If error is very small (within 2 degrees), stop moving
        // This prevents constant micro-adjustments
        if (Math.abs(error) < Math.toRadians(2)) {
            power = 0;
        }

        // STEP 9: Send power to motor
        turretMotor.setPower(power);
    }

    // ==================== EXISTING METHODS ====================

    /**
     * Scales motor power to prevent exceeding safe limits
     * Limits power to ±0.8 to protect motors
     */
    private double ScalePower(double power) {
        return Math.max(-0.8, Math.min(0.8, power));
    }

    /**
     * Controls the drivetrain using gamepad joysticks
     * RIGHT STICK Y: Forward/backward
     * RIGHT STICK X: Turn left/right
     * LEFT STICK X: Strafe left/right
     */
    private void moveDriveTrain() {
        // Read joystick values (negative because Y is inverted on gamepad)
        double forward = -gamepad1.right_stick_y;  // Forward/backward
        double turn = -gamepad1.right_stick_x;     // Rotation
        double strafe = gamepad1.left_stick_x;     // Left/right strafe

        // Calculate power for each wheel using mecanum drive equations
        double fl = forward + strafe + turn;  // Front left
        double fr = forward - strafe - turn;  // Front right
        double bl = forward - strafe + turn;  // Back left
        double br = forward + strafe - turn;  // Back right

        // Apply power to motors with scaling
        FLeft.setPower(ScalePower(fl));
        Fright.setPower(ScalePower(fr));
        BLeft.setPower(ScalePower(bl));
        Bright.setPower(ScalePower(br));
    }

    /**
     * Toggles intake motor on/off
     * RIGHT TRIGGER: Toggle intake
     */
    private void handleIntake() {
        // Check if trigger is pressed (threshold 0.5)
        boolean triggerPressed = gamepad1.right_trigger > 0.5;

        // If trigger just pressed, toggle intake state
        if (triggerPressed && !lastTriggerPressed) {
            intakeOn = !intakeOn;
        }

        // Set motor power based on state
        if (intakeOn) {
            intake.setPower(1.0);      // Run intake motor
        } else {
            intake.setPower(0.0);      // Stop intake motor
        }

        lastTriggerPressed = triggerPressed;
    }

    /**
     * Toggles outtake motors on/off using velocity control
     * B BUTTON: Toggle outtake
     */
    private void handleOuttakeToggle() {
        boolean bPressed = gamepad1.b;

        // If B just pressed, toggle outtake state
        if (bPressed && !lastOuttakeTogglePressed) {
            outtakeOn = !outtakeOn;
        }

        // Use velocity control instead of power control
        if (outtakeOn) {
            // Set target velocity in ticks per second
            // The PIDF controller will maintain this velocity even under load
            outtake.setVelocity(targetVelocity);
            outtake2.setVelocity(targetVelocity);
        } else {
            outtake.setVelocity(0);
            outtake2.setVelocity(0);
        }

        lastOuttakeTogglePressed = bPressed;
    }

    /**
     * Adjusts outtake target velocity
     * DPAD RIGHT: Increase velocity by 50 ticks/sec (fine control)
     * DPAD LEFT: Decrease velocity by 50 ticks/sec (fine control)
     * RIGHT BUMPER: Increase velocity by 200 ticks/sec (coarse control)
     * (LEFT BUMPER is used for turret, so only right bumper for large adjustments)
     */
    private void handleOuttakeVelocityAdjust() {
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;
        boolean rightBumper = gamepad1.right_bumper;

        // Fine adjustment: ±50 ticks/sec
        if (right && !lastDpadRight) {
            targetVelocity = Math.min(targetVelocity + 50, 3000);
        }
        if (left && !lastDpadLeft) {
            targetVelocity = Math.max(targetVelocity - 50, 0);
        }

        // Coarse adjustment: +200 ticks/sec (only increase with right bumper)
        // Note: Left bumper is reserved for turret auto-aim toggle
        if (rightBumper && !lastDpadRight) {  // Reuse lastDpadRight to prevent spam
            targetVelocity = Math.min(targetVelocity + 200, 3000);
        }

        lastDpadRight = right || rightBumper;  // Track both for debouncing
        lastDpadLeft = left;

        // Update motors if outtake is running
        if (outtakeOn) {
            outtake.setVelocity(targetVelocity);
            outtake2.setVelocity(targetVelocity);
        }
    }
}