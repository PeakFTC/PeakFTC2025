package org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * TeleOp with STOP-THEN-AIM Turret Control + LOCK MODE
 *
 * KEY FEATURES:
 * 1. Added constant 3-4 degree LEFT compensation for systematic aiming error
 * 2. Robot accepts movement inputs even while turret is aiming (no lockout)
 * 3. Turret aiming happens in parallel with driving
 * 4. LOCK MODE: Press X to lock robot in place and resist pushing
 *
 * CONTROLS:
 * - LEFT BUMPER: Toggle turret auto-aim on/off
 * - RIGHT BUMPER: Quick increase outtake velocity (+200)
 * - RIGHT TRIGGER: Toggle intake forward
 * - LEFT TRIGGER: Intake reverse
 * - A BUTTON: Close stopper servo
 * - B BUTTON: Toggle outtake on/off
 * - Y BUTTON: Open stopper servo
 * - X BUTTON: Toggle LOCK MODE (robot resists pushing)
 * - DPAD RIGHT/LEFT: Adjust outtake velocity
 * - RIGHT STICK: Drive forward/backward and turn
 * - LEFT STICK: Strafe left/right
 */
@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends OpMode {

    // ==================== HARDWARE ====================
    private Follower follower;  // Pedro Pathing follower for lock mode
    private DcMotor intake;
    private DcMotorEx outtake;
    private DcMotorEx outtake2;
    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;
    private ElapsedTime time;
    private Servo stopper;
    private DcMotorEx turretMotor;
    private GoBildaPinpointDriver pinpoint;

    // ==================== FIELD CONSTANTS ====================
    static final double BLUE_GOAL_X = 0;
    static final double BLUE_GOAL_Y = 144;

    // ==================== TURRET CONSTANTS ====================
    static final double MAX_TURRET_ANGLE = Math.toRadians(90);
    static final double TICKS_PER_RADIAN = 428;

    static final double kP_TURRET = 3.0;
    static final double MAX_TURRET_POWER = 0.6;
    static final double TURRET_DEADBAND_DEGREES = 0.3;

    // SYSTEMATIC AIMING COMPENSATION
    static final double COMPENSATION_DEGREES = 3.5;

    // ==================== STOP DETECTION CONSTANTS ====================
    static final double VELOCITY_STOPPED_THRESHOLD = 2.0;  // inches/sec
    static final double HEADING_RATE_STOPPED_THRESHOLD = 0.1;  // rad/sec
    static final double SETTLE_TIME = 0.15;  // seconds

    // ==================== SERVO CONSTANTS ====================
    static final double STOPPER_OPEN = 0.25;
    static final double STOPPER_CLOSE = 0.38;

    // ==================== ROBOT STATE ====================
    double robotX = 56;
    double robotY = 8;
    double robotHeading = Math.toRadians(90);

    // ==================== MOVEMENT DETECTION ====================
    private double lastRobotX = 56;
    private double lastRobotY = 8;
    private double lastRobotHeading = Math.toRadians(90);
    private boolean robotIsStopped = false;
    private double timeStoppedAt = 0;
    private boolean hasSettled = false;

    // ==================== TURRET CONTROL ====================
    boolean turretAutoAimEnabled = false;
    private boolean lastLeftBumper = false;

    // ==================== LOCK MODE ====================
    private boolean lockMode = false;
    private boolean xButtonPressed = false;
    private Pose lockPosition;
    private double lockHeading;

    // ==================== EXISTING VARIABLES ====================
    private boolean lastTriggerPressed = false;
    private boolean lastOuttakeTogglePressed = false;
    private boolean lastAPressed = false;
    private boolean lastYPressed = false;
    private boolean intakeOn = false;
    private boolean outtakeOn = false;
    private double targetVelocity = 1276;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastRightBumper = false;

    // ==================== PIDF COEFFICIENTS ====================
    private static final double kP_VELOCITY = 12.0;
    private static final double kI_VELOCITY = 1.8;
    private static final double kD_VELOCITY = 1.8;
    private static final double kF_VELOCITY = 13.2;

    //======================= Color sensor========================
    private colorSensorPeak colorSensor;
    private ElapsedTime outTakeTimer ;
    private double outakeWaitStart;
    private final double outtakeWaitTime = 500;

    @Override
    public void init() {
        time = new ElapsedTime();
        outTakeTimer = new ElapsedTime();
        // ==================== HARDWARE INIT ====================
        // Initialize Pedro follower for lock mode
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        FLeft = hardwareMap.dcMotor.get("FL");
        BLeft = hardwareMap.dcMotor.get("BL");
        Fright = hardwareMap.dcMotor.get("FR");
        Bright = hardwareMap.dcMotor.get("BR");
        stopper = hardwareMap.servo.get("stopper");
        stopper.setPosition(STOPPER_OPEN);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(190.5, -190.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90);
        pinpoint.setPosition(startPose);
        pinpoint.recalibrateIMU();
        pinpoint.update();

        Pose2D verifyPose = pinpoint.getPosition();
        robotX = verifyPose.getX(DistanceUnit.INCH);
        robotY = verifyPose.getY(DistanceUnit.INCH);
        robotHeading = verifyPose.getHeading(AngleUnit.RADIANS);

        // Set starting pose for follower
        follower.setStartingPose(new Pose(robotX, robotY, robotHeading));

        telemetry.addData("Mode", "STOP-THEN-AIM + LOCK MODE");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
        telemetry.update();

        // ==================== MOTOR DIRECTIONS ====================
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ==================== OUTTAKE SETUP ====================
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);
        outtake2.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);

        //=========================Color Sensor===============================
        colorSensor = new colorSensorPeak(hardwareMap,telemetry);
        colorSensor.initColorSensorPeak();

        telemetry.addData("Status", "Ready - LOCK MODE ENABLED");
        telemetry.update();
    }

    @Override
    public void start() {
        time.reset();
        turretAutoAimEnabled = true;

        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotHeading = robotHeading;
    }

    @Override
    public void loop() {
        // ==================== UPDATE ODOMETRY ====================
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        robotX = pos.getX(DistanceUnit.INCH);
        robotY = pos.getY(DistanceUnit.INCH);
        robotHeading = pos.getHeading(AngleUnit.RADIANS);

        // Update follower (ALWAYS - needed for lock mode and odometry)
        follower.update();

        // ==================== X BUTTON - TOGGLE LOCK MODE ====================
        if (gamepad1.x && !xButtonPressed) {
            lockMode = !lockMode;

            if (lockMode) {
                // ENTERING LOCK MODE - Save current position
                Pose currentPose = follower.getPose();
                lockPosition = currentPose;
                lockHeading = currentPose.getHeading();

                // Tell follower to hold this position
                follower.holdPoint(new BezierPoint(lockPosition), lockHeading);

                // Rumble controller to confirm lock
                gamepad1.rumble(200);
            } else {
                // EXITING LOCK MODE - Stop holding
                follower.breakFollowing();

                // Rumble twice to confirm unlock
                gamepad1.rumble(100);
            }

            xButtonPressed = true;
        } else if (!gamepad1.x) {
            xButtonPressed = false;
        }

        // ==================== DETECT IF ROBOT IS STOPPED ====================
        detectRobotStopped();

        // ==================== CONTROLS ====================
        handleIntake();
        handleOuttakeToggle();
        handleOuttakeVelocityAdjust();
        handleStopperServo();

        if (!lockMode) {
            // Normal driving when not locked
            moveDriveTrain();
        }
        // If lockMode is true, follower.update() handles position holding

        handleTurretAutoAim();

        // ==================== UPDATE TRACKING ====================
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotHeading = robotHeading;

        // ==================== TELEMETRY ====================
        telemetry.addData("=== LOCK MODE ===", lockMode ? "LOCKED 🔒" : "UNLOCKED");
        telemetry.addData("", "");

        telemetry.addData("=== ROBOT STATUS ===", "");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));

        if (lockMode) {
            telemetry.addData("Lock Position", "X: %.1f, Y: %.1f", lockPosition.getX(), lockPosition.getY());
            telemetry.addData("Lock Heading", "%.0f°", Math.toDegrees(lockHeading));
            telemetry.addData("Position Error", "%.2f inches",
                    Math.sqrt(Math.pow(robotX - lockPosition.getX(), 2) + Math.pow(robotY - lockPosition.getY(), 2)));
        }

        // Show movement status
        String movementStatus;
        if (lockMode) {
            movementStatus = "LOCKED - Resisting push!";
        } else if (robotIsStopped) {
            if (hasSettled) {
                movementStatus = "STOPPED & SETTLED - Aiming!";
            } else {
                double timeWaiting = time.seconds() - timeStoppedAt;
                movementStatus = String.format("STOPPED - Settling... %.2fs", timeWaiting);
            }
        } else {
            movementStatus = "MOVING - Turret on standby";
        }
        telemetry.addData("Movement", movementStatus);

        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Auto-Aim", turretAutoAimEnabled ? "ON" : "OFF (L-Bumper)");
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(turretMotor.getCurrentPosition() / TICKS_PER_RADIAN));

        telemetry.addData("=== OUTTAKE ===", "");
        telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
        telemetry.addData("Velocity", "%.0f / %.0f ticks/sec", outtake.getVelocity(), targetVelocity);
        telemetry.addData("Stopper", stopper.getPosition() > 0.30 ? "CLOSED" : "OPEN");

        telemetry.update();
    }

    /**
     * Detects when robot has stopped moving
     */
    private void detectRobotStopped() {
        // Calculate velocities
        double deltaX = robotX - lastRobotX;
        double deltaY = robotY - lastRobotY;
        double translationalVelocity = Math.sqrt(deltaX * deltaX + deltaY * deltaY) / 0.02;

        double headingChange = robotHeading - lastRobotHeading;
        while (headingChange > Math.PI) headingChange -= 2.0 * Math.PI;
        while (headingChange < -Math.PI) headingChange += 2.0 * Math.PI;
        double rotationalVelocity = Math.abs(headingChange / 0.02);

        boolean currentlyStopped = (translationalVelocity < VELOCITY_STOPPED_THRESHOLD) &&
                (rotationalVelocity < HEADING_RATE_STOPPED_THRESHOLD);

        if (currentlyStopped && !robotIsStopped) {
            robotIsStopped = true;
            timeStoppedAt = time.seconds();
            hasSettled = false;
        } else if (!currentlyStopped && robotIsStopped) {
            robotIsStopped = false;
            hasSettled = false;
        } else if (robotIsStopped && !hasSettled) {
            if (time.seconds() - timeStoppedAt >= SETTLE_TIME) {
                hasSettled = true;
            }
        }
    }

    /**
     * Only aims when robot is stopped and settled
     */
    private void handleTurretAutoAim() {
        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            turretAutoAimEnabled = !turretAutoAimEnabled;
        }
        lastLeftBumper = leftBumper;

        if (!turretAutoAimEnabled) {
            turretMotor.setPower(0);
            return;
        }

        if (!robotIsStopped || !hasSettled) {
            turretMotor.setPower(0);
            return;
        }

        aimTurretAtGoal();
    }

    /**
     * Applies constant systematic compensation
     */
    private void aimTurretAtGoal() {
        double deltaX = BLUE_GOAL_X - robotX;
        double deltaY = BLUE_GOAL_Y - robotY;
        double fieldAngleToGoal = Math.atan2(deltaY, deltaX);

        double turretAngleNeeded = fieldAngleToGoal - robotHeading;

        while (turretAngleNeeded > Math.PI) turretAngleNeeded -= 2.0 * Math.PI;
        while (turretAngleNeeded < -Math.PI) turretAngleNeeded += 2.0 * Math.PI;

        double turretAngleRaw = turretAngleNeeded;

        double compensationRadians = Math.toRadians(COMPENSATION_DEGREES);
        turretAngleNeeded += compensationRadians;

        double turretAngleClamped = Range.clip(turretAngleNeeded, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

        boolean hitLimit = (turretAngleClamped != turretAngleNeeded);

        int currentEncoderTicks = turretMotor.getCurrentPosition();
        double currentTurretAngle = currentEncoderTicks / TICKS_PER_RADIAN;
        double angleError = turretAngleClamped - currentTurretAngle;

        while (angleError > Math.PI) angleError -= 2.0 * Math.PI;
        while (angleError < -Math.PI) angleError += 2.0 * Math.PI;

        double motorPower = angleError * kP_TURRET;
        motorPower = Range.clip(motorPower, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        if (Math.abs(angleError) < Math.toRadians(TURRET_DEADBAND_DEGREES)) {
            motorPower = 0.0;
            telemetry.addData("TURRET STATUS", "✓ ON TARGET!");
        }

        if (hitLimit) {
            if ((turretAngleClamped >= MAX_TURRET_ANGLE && motorPower > 0) ||
                    (turretAngleClamped <= -MAX_TURRET_ANGLE && motorPower < 0)) {
                motorPower = 0;
                telemetry.addData("TURRET STATUS", "⚠ AT LIMIT");
            }
        }

        turretMotor.setPower(motorPower);

        telemetry.addData("=== AIM CALCULATION ===", "");
        telemetry.addData("Distance to Goal", "%.1f inches", Math.sqrt(deltaX*deltaX + deltaY*deltaY));
        telemetry.addData("Field Angle", "%.1f°", Math.toDegrees(fieldAngleToGoal));
        telemetry.addData("Turret Needed (raw)", "%.1f°", Math.toDegrees(turretAngleRaw));
        telemetry.addData("Compensation", "+%.1f°", COMPENSATION_DEGREES);
        telemetry.addData("Turret Needed (comp)", "%.1f°", Math.toDegrees(turretAngleClamped));
        telemetry.addData("Current Turret", "%.1f°", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Error", "%.2f°", Math.toDegrees(angleError));
        telemetry.addData("Motor Power", "%.3f", motorPower);
    }

    private void handleStopperServo() {
        boolean aPressed = gamepad1.a;
        boolean yPressed = gamepad1.y;
        if (aPressed && !lastAPressed) stopper.setPosition(STOPPER_CLOSE);
        if (yPressed && !lastYPressed) stopper.setPosition(STOPPER_OPEN);
        lastAPressed = aPressed;
        lastYPressed = yPressed;
    }

    private double ScalePower(double power) {
        return Math.max(-0.8, Math.min(0.8, power));
    }

    private void moveDriveTrain() {
        double forward = -gamepad1.right_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        FLeft.setPower(ScalePower(fl));
        Fright.setPower(ScalePower(fr));
        BLeft.setPower(ScalePower(bl));
        Bright.setPower(ScalePower(br));
    }

    private void handleIntake() {
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        if (rightTriggerPressed && !lastTriggerPressed) {
            intakeOn = !intakeOn;
        }
        lastTriggerPressed = rightTriggerPressed;

        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

        if (leftTriggerPressed) {
            intake.setPower(-1.0);
        } else if (intakeOn) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    private boolean outEmptyFirst=false;
    private void handleOuttakeToggle() {
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastOuttakeTogglePressed) {
            outtakeOn = !outtakeOn;
        }

        if (outtakeOn) {
            outtake.setVelocity(targetVelocity);
            outtake2.setVelocity(targetVelocity);
            if(colorSensor.isOuttakeEmpty()) {
                if(!outEmptyFirst) {
                    outTakeTimer.reset();
                    outakeWaitStart = outTakeTimer.milliseconds();
                    outEmptyFirst = true;
                }
                if( outTakeTimer.milliseconds() -outakeWaitStart > outtakeWaitTime){
                    outtakeOn=false;
                    outEmptyFirst=false;
                    intakeOn =false;
                    stopper.setPosition(STOPPER_CLOSE);
                }
            }
        } else {
            outtake.setVelocity(0);
            outtake2.setVelocity(0);
        }

        lastOuttakeTogglePressed = bPressed;
    }

    private void handleOuttakeVelocityAdjust() {
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;
        boolean rightBumper = gamepad1.right_bumper;

        if (right && !lastDpadRight) targetVelocity = Math.min(targetVelocity + 50, 3000);
        if (left && !lastDpadLeft) targetVelocity = Math.max(targetVelocity - 50, 0);
        if (rightBumper && !lastRightBumper) targetVelocity = Math.min(targetVelocity + 200, 3000);

        lastDpadRight = right;
        lastDpadLeft = left;
        lastRightBumper = rightBumper;

        if (outtakeOn) {
            outtake.setVelocity(targetVelocity);
            outtake2.setVelocity(targetVelocity);
        }
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.setPower(0);
        outtake.setVelocity(0);
        outtake2.setVelocity(0);
        turretMotor.setPower(0);
    }
}