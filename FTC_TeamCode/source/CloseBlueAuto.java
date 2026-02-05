package org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Close Blue - Full Auto", group = "Competition")
public class CloseBlueAuto extends LinearOpMode {
    private ElapsedTime timer;
    // ==================== HARDWARE ====================
    private Follower follower;
    private DcMotor intake;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotorEx turretMotor;
    private Servo stopper;

    // ==================== PATHS ====================
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    // ==================== CONSTANTS ====================
    // Field coordinates
    static final double BLUE_GOAL_X = 0;
    static final double BLUE_GOAL_Y = 144;

    // Turret constants
    static final double TICKS_PER_RADIAN = 428;
    static final double MAX_TURRET_ANGLE = Math.toRadians(90);

    // Outtake constants
    static final double STOPPER_OPEN = 0.25;
    static final double STOPPER_CLOSE = 0.38;
    static final double PRELOAD_VELOCITY = 1235;  // Matching user's tested value
    static final double SAMPLE_VELOCITY = 1275;   // Matching user's tested value
    // Moderate PIDF - fast recovery without overshooting
    static final double kP_VELOCITY = 16.0;  // Moderate increase from 12.0
    static final double kI_VELOCITY = 2.5;   // Moderate increase from 1.8
    static final double kD_VELOCITY = 2.3;   // Moderate increase fzrom 1.8
    static final double kF_VELOCITY = 13.2;

    // Timing constants
    static final double SPINUP_TIME = 2.0;
    static final double SHOOT_TIME = 2.0;  // Matching user's tested value
    static final double INTAKE_HOLD_POWER = 0.1;
    static final double INTAKE_FULL_POWER = 1.0;

    // ==================== FIXED TURRET POSITION ====================
    // Starting position: (20.500, 122.500) facing 143°
    // Shooting position: (56.500, 86.000) facing 135°
    static final double START_X = 20.500;
    static final double START_Y = 122.500;
    static final double START_HEADING = Math.toRadians(143);

    static final double SHOOT_X = 56.500;
    static final double SHOOT_Y = 86.000;
    static final double SHOOT_HEADING = Math.toRadians(135);

    // Calculate ONE fixed turret angle for ALL shooting (from shooting position)
    static final double SHOOTING_TURRET_ANGLE = calculateTurretAngle(SHOOT_X, SHOOT_Y, SHOOT_HEADING);

    // Home position (turret facing straight forward)
    static final double TURRET_HOME_ANGLE = 0;

    // ==================Color Sensor==========================
   colorSensorPeak colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // ==================== INITIALIZATION ====================
        initializeHardware();
        buildPaths();
        timer = new ElapsedTime();
        //============Color Sensor initialization
        colorSensor = new colorSensorPeak(hardwareMap,telemetry);
        colorSensor.initColorSensorPeak();

        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));

        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Shooting Turret Angle", "%.1f°", Math.toDegrees(SHOOTING_TURRET_ANGLE));
        telemetry.update();

        waitForStart();

        // ==================== AUTONOMOUS SEQUENCE ====================

        // ========== PATH 1: Move to shooting position ==========
        telemetry.addData("Action", "Path 1 - Moving to shooting position");
        telemetry.update();

        follower.followPath(Path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT PRELOAD (after reaching shooting position) ==========
        telemetry.addData("Action", "Shooting preload");
        telemetry.update();
        shootPreload();

        startFlywheels();  // START IMMEDIATELY for first cycle

        // ========== PATH 2: Intake first sample (REVERSED) - INTAKE ON ==========
        telemetry.addData("Action", "Path 2 - Intaking first sample");
        telemetry.update();

        setIntake(true);  // Turn on intake at START of path

        follower.followPath(Path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        setIntakeHold();  // Path 2 complete - switch to hold mode
        // Flywheels already spinning from after preload shot
        aimFirstSample();  // Aim with +17° left offset (first sample needs extra)
        sleep(50);

        // ========== PATH 3: Return to shoot - FLYWHEELS ALREADY SPINNING ==========
        telemetry.addData("Action", "Path 3 - Returning to shoot");
        telemetry.update();

        follower.followPath(Path3);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT FIRST SAMPLE ==========
        telemetry.addData("Action", "Shooting first sample");
        telemetry.update();
        shootSample();

        startFlywheels();  // START IMMEDIATELY for next cycle

        // ========== PATH 4: Intake second sample (REVERSED) - INTAKE ON ==========
        telemetry.addData("Action", "Path 4 - Intaking second sample");
        telemetry.update();

        setIntake(true);  // Turn on intake at START of path

        follower.followPath(Path4);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
        intakeWaitTime();

        setIntakeHold();  // Path 4 complete - switch to hold mode
        // Flywheels already spinning from after previous shot
        aimSample();  // Aim with +10° left offset
        sleep(50);

        // ========== PATH 5: Return to shoot - FLYWHEELS ALREADY SPINNING ==========
        telemetry.addData("Action", "Path 5 - Returning to shoot");
        telemetry.update();

        follower.followPath(Path5);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT SECOND SAMPLE ==========
        telemetry.addData("Action", "Shooting second sample");
        telemetry.update();
        shootSample();
/*
        startFlywheels();  // START IMMEDIATELY for next cycle

        // ========== PATH 6: Intake third sample (REVERSED) - INTAKE ON ==========
        telemetry.addData("Action", "Path 6 - Intaking third sample");
        telemetry.update();

        setIntake(true);  // Turn on intake at START of path

        follower.followPath(Path6);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
        sleep(500);

        setIntakeHold();  // Path 6 complete - switch to hold mode
        // Flywheels already spinning from after previous shot
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);  // Aim turret now
        sleep(50);

        // ========== PATH 7: Final return to shoot - FLYWHEELS ALREADY SPINNING ==========
        telemetry.addData("Action", "Path 7 - Final return to shoot");
        telemetry.update();

        follower.followPath(Path7);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT THIRD SAMPLE ==========
        telemetry.addData("Action", "Shooting third sample");
        telemetry.update();
        shootSample();

        // ========== COMPLETE ==========
        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();

        // Stop all mechanisms and reset turret to home
        stopAllMechanisms();
        setTurretToFixedAngle(TURRET_HOME_ANGLE);
        sleep(200);  // REDUCED from 500ms

 */
    }

    void intakeWaitTime()
    {
        timer.reset();
        double waitTime = timer.milliseconds();
        while(!colorSensor.isIntakeFull() && timer.milliseconds()-waitTime <1000);

    }

    // ==================== SHOOTING METHODS ====================

    private void shootPreload() {
        // Wait briefly to ensure robot is fully stopped
        sleep(100);

        // 1. Aim turret at fixed angle (no offset for preload)
        aimPreload();

        // 2. Close stopper
        stopper.setPosition(STOPPER_CLOSE);

        // 3. Spin up flywheels to PRELOAD velocity
        outtake1.setVelocity(PRELOAD_VELOCITY);
        outtake2.setVelocity(PRELOAD_VELOCITY);

        // 4. Wait for spinup
        sleep((long)(SPINUP_TIME * 1000));

        // 5. Open stopper and turn on intake to feed balls
        stopper.setPosition(STOPPER_OPEN);
        sleep(50);
        setIntakeFull();
        timer.reset();
        double waitTime = timer.milliseconds();
        while(!colorSensor.isOuttakeEmpty() || (timer.milliseconds() -waitTime < 2500));
        // 6. Run for shoot duration
       // sleep((long)(SHOOT_TIME * 1000));
        sleep(500); // addition wait for last ball shoot

        // 7. Stop everything
        setIntake(false);
        stopper.setPosition(STOPPER_CLOSE);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
    }

    private void shootSample() {
        // Flywheels have been spinning for ENTIRE intake + return path (plenty of time!)
        // Intake currently at HOLD power (0.1)

        // 1. Make sure stopper is closed
        stopper.setPosition(STOPPER_CLOSE);
        sleep(100);

        // 2. Flywheels are already at full speed - just verify
        // (They've been spinning for ~3-5 seconds already)

        // 3. RAMP UP intake to full power for shooting
        setIntakeFull();
        sleep(200);

        // 4. Open stopper to shoot
        stopper.setPosition(STOPPER_OPEN);
        sleep(50);

        // 5. Run for shoot duration
        timer.reset();
        double waitTime = timer.milliseconds();
        while(!colorSensor.isOuttakeEmpty() || (timer.milliseconds() -waitTime < 2500));
        sleep((long)(SHOOT_TIME * 500));

        // 6. Stop everything
        setIntake(false);
        stopper.setPosition(STOPPER_CLOSE);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
    }

    private void startFlywheels() {
        outtake1.setVelocity(SAMPLE_VELOCITY);
        outtake2.setVelocity(SAMPLE_VELOCITY);
    }

    private void setTurretToFixedAngle(double angleRadians) {
        double clampedAngle = Math.max(-MAX_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleRadians));
        int targetTicks = (int)(clampedAngle * TICKS_PER_RADIAN);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);

        telemetry.addData("Turret Target", "%.1f° (%d ticks)", Math.toDegrees(clampedAngle), targetTicks);
    }

    // ==================== HELPER METHODS ====================

    private static double calculateTurretAngle(double robotX, double robotY, double robotHeading) {
        double deltaX = BLUE_GOAL_X - robotX;
        double deltaY = BLUE_GOAL_Y - robotY;
        double fieldAngleToGoal = Math.atan2(deltaY, deltaX);

        double turretAngle = fieldAngleToGoal - robotHeading;

        while (turretAngle > Math.PI) turretAngle -= 2.0 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

        return turretAngle;
    }

    private void aimPreload() {
        // Use fixed shooting angle for preload (no offset)
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);
    }

    private void aimFirstSample() {
        // Use fixed shooting angle + 17 degrees LEFT for first sample shot
        double compensatedAngle = SHOOTING_TURRET_ANGLE + Math.toRadians(17);
        setTurretToFixedAngle(compensatedAngle);

        telemetry.addData("First Sample Aim", "%.2f° (+17° left)", Math.toDegrees(compensatedAngle));
        telemetry.update();
    }

    private void aimSample() {
        // Use fixed shooting angle + 10 degrees LEFT for other sample shots
        double compensatedAngle = SHOOTING_TURRET_ANGLE + Math.toRadians(10);
        setTurretToFixedAngle(compensatedAngle);

        telemetry.addData("Sample Aim", "%.2f° (+10° left)", Math.toDegrees(compensatedAngle));
        telemetry.update();
    }

    private void setIntake(boolean on) {
        intake.setPower(on ? INTAKE_FULL_POWER : 0.0);
    }

    private void setIntakeHold() {
        intake.setPower(INTAKE_HOLD_POWER);
    }

    private void setIntakeFull() {
        intake.setPower(INTAKE_FULL_POWER);
    }

    private void stopAllMechanisms() {
        setIntake(false);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
        turretMotor.setPower(0);
        stopper.setPosition(STOPPER_CLOSE);
    }

    private void updateTelemetry() {
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Intake", intake.getPower() > 0 ? "ON" : "OFF");
        telemetry.addData("Flywheel", "%.0f tps", outtake1.getVelocity());
        telemetry.update();
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.dcMotor.get("intake");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        stopper = hardwareMap.servo.get("stopper");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);
        outtake2.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);

        stopper.setPosition(STOPPER_CLOSE);
    }

    private void buildPaths() {
        // Path1: Move from start to shooting position
        Path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(20.500, 122.500),
                        new Pose(51.000, 108.000),
                        new Pose(56.500, 86.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        // Path2: REVERSED intake first sample
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.500, 86.000),
                        new Pose(60.000, 78.500),
                        new Pose(15.000, 84.000)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Path3: Return to shoot first sample
        Path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 84.000),
                        new Pose(16.000, 70.000),
                        new Pose(56.500, 86.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(355), Math.toRadians(135))
                .build();

        // Path4: REVERSED intake second sample
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.500, 86.000),
                        new Pose(68.000, 58.000),
                        new Pose(15.000, 60.000)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Path5: Return to shoot second sample
        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(15.000, 60.000),
                        new Pose(60.000, 60.000),
                        new Pose(57.000, 86.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();
/*
        // Path6: REVERSED intake third sample
        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57.000, 86.000),
                        new Pose(74.000, 32.000),
                        new Pose(18.500, 35.500)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Path7: Return to shoot third sample
        Path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(18.500, 35.500),
                        new Pose(45.000, 32.500),
                        new Pose(57.000, 86.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

 */
    }
}