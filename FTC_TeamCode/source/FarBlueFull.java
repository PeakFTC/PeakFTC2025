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

@Autonomous(name = "Far Blue - Full Auto", group = "Competition")
public class FarBlueFull extends LinearOpMode {

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
    public PathChain Path8;
    public PathChain Path9;

    // ==================== CONSTANTS ====================
            // Field coordinates
            static final double BLUE_GOAL_X = 0;
    static final double BLUE_GOAL_Y = 144;

    // Turret constants
            static final double TICKS_PER_RADIAN = 428;
    static final double MAX_TURRET_ANGLE = Math.toRadians(90);

    // Outtake constants
            static final double STOPPER_OPEN = 0.25;    // UPDATED - tested position (not blocking)
                static final double STOPPER_CLOSE = 0.38;   // UPDATED - tested position (blocking)
                static final double PRELOAD_VELOCITY = 1370;  // LOWER power for preload (too strong before)
                static final double SAMPLE_VELOCITY = 1420;   // HIGHER power for intaked samples
                // INCREASED PIDF for faster recovery between balls in rapid-fire sequence
                static final double kP_VELOCITY = 12.0;  // INCREASED from 12.0 - faster response
                static final double kI_VELOCITY = 1.8;   // INCREASED from 1.8 - fights sustained error
                static final double kD_VELOCITY = 1.8;   // INCREASED from 1.8 - faster reaction
                static final double kF_VELOCITY = 13.2;

    // Timing constants
            static final double SPINUP_TIME = 2.0;      // Time for flywheels to reach speed
                static final double SHOOT_TIME = 3.0;       // Time to shoot all balls (intake runs)
                static final double INTAKE_HOLD_POWER = 0.1;  // Low power to hold ball in place
                static final double INTAKE_FULL_POWER = 1.0;  // Full power for intaking/shooting

                // ==================== FIXED TURRET POSITION ====================
                // Starting position: (58, 9) facing 90°
                static final double SHOOT_X = 58;
    static final double SHOOT_Y = 9;
    static final double ROBOT_HEADING = Math.toRadians(90);

    // Calculate ONE fixed turret angle for ALL shooting
            static final double SHOOTING_TURRET_ANGLE = calculateTurretAngle(SHOOT_X, SHOOT_Y, ROBOT_HEADING);

    // Home position (turret facing straight forward)
            static final double TURRET_HOME_ANGLE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ==================== INITIALIZATION ====================
        initializeHardware();
        buildPaths();

        follower.setStartingPose(new Pose(58.000, 9.000, Math.toRadians(90)));

        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Shooting Turret Angle", "%.1f°", Math.toDegrees(SHOOTING_TURRET_ANGLE));
        telemetry.update();

        waitForStart();

        // ==================== AUTONOMOUS SEQUENCE ====================

        // ========== SHOOT PRELOAD ==========
        telemetry.addData("Action", "Shooting preload");
        telemetry.update();
        shootPreload();

        // ========== PATH 1: Move to first sample ==========
        telemetry.addData("Action", "Path 1 - Moving to first sample");
        telemetry.update();

        // Start spinning up flywheels for next shot
        startFlywheels();

        follower.followPath(Path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== PATH 2: Intake first sample (REVERSED) ==========
        telemetry.addData("Action", "Path 2 - Intaking sample");
        telemetry.update();

        setIntake(true);

        follower.followPath(Path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
        sleep(500);
        setIntakeHold();  // CHANGED: Keep at low power to hold ball instead of turning off

        // ========== PATH 3: Return to shoot ==========
        telemetry.addData("Action", "Path 3 - Returning to shoot");
        telemetry.update();

        // Set turret to shooting position
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);

        follower.followPath(Path3);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT FIRST SAMPLE ==========
        telemetry.addData("Action", "Shooting first sample");
        telemetry.update();
        shootSample();

        startFlywheels();  // MOVED HERE - start spinning immediately for next cycle

        // ========== PATH 4: Move to second sample ==========
        telemetry.addData("Action", "Path 4 - Moving to second sample");
        telemetry.update();

        follower.followPath(Path4);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== PATH 5: Intake second sample (REVERSED) ==========
        telemetry.addData("Action", "Path 5 - Intaking second sample");
        telemetry.update();

        setIntake(true);

        follower.followPath(Path5);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
        sleep(500);
        setIntakeHold();  // CHANGED: Keep at low power to hold ball instead of turning off

        // ========== PATH 6: Return to shoot ==========
        telemetry.addData("Action", "Path 6 - Returning to shoot");
        telemetry.update();

        // Set turret to shooting position
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);

        follower.followPath(Path6);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== SHOOT SECOND SAMPLE ==========
        telemetry.addData("Action", "Shooting second sample");
        telemetry.update();
        shootSample();

        startFlywheels();  // MOVED HERE - start spinning immediately for next cycle

        // ========== PATH 7: Move to third sample ==========
        telemetry.addData("Action", "Path 7 - Moving to third sample");
        telemetry.update();

        follower.followPath(Path7);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }

        // ========== PATH 8: Intake third sample (REVERSED) ==========
        telemetry.addData("Action", "Path 8 - Intaking third sample");
        telemetry.update();

        setIntake(true);

        follower.followPath(Path8);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
        sleep(500);

        setIntakeHold();  // CHANGED: Keep at low power to hold ball instead of turning off

        // ========== PATH 9: Final return to shoot ==========
        telemetry.addData("Action", "Path 9 - Final return to shoot");
        telemetry.update();

        // Set turret to shooting position
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);

        follower.followPath(Path9);
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
    }

    // ==================== SHOOTING METHODS ====================

    private void shootPreload() {
        // 1. Set turret to fixed shooting angle
        setTurretToFixedAngle(SHOOTING_TURRET_ANGLE);

        // 3. Spin up flywheels to PRELOAD velocity (lower power)
        outtake1.setVelocity(PRELOAD_VELOCITY);
        outtake2.setVelocity(PRELOAD_VELOCITY);

        // 4. Wait for spinup - EXTRA TIME for first shot
        sleep(500);  // INCREASED from 2000ms - give preload more time

        // 5. Open stopper and turn on intake to feed balls
        stopper.setPosition(STOPPER_OPEN);
        telemetry.addData("Stopper", "OPEN - %.2f", STOPPER_OPEN);
        telemetry.update();

        // 6. PULSED SHOOTING - feed balls with pauses for velocity recovery
        // Shoot All Balls
        setIntakeFull();
        sleep(1500);  // Feed All Balls

        // 7. Stop everything
        setIntake(false);
        stopper.setPosition(STOPPER_CLOSE);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);

    }

    private void shootSample() {
        // Turret already positioned and flywheels already spinning
        // Intake currently at HOLD power (0.1)
        // 2. Set flywheels to SAMPLE velocity (higher power for intaked balls)
        outtake1.setVelocity(SAMPLE_VELOCITY);
        outtake2.setVelocity(SAMPLE_VELOCITY);
        sleep(500);  // REDUCED from 500ms


        // 4. Open stopper to shoot
        stopper.setPosition(STOPPER_OPEN);
        sleep(100);
        telemetry.addData("Stopper", "OPEN - %.2f", STOPPER_OPEN);
        telemetry.addData("Flywheel Velocity", "%.0f tps", outtake1.getVelocity());
        telemetry.update();

        // 5. PULSED SHOOTING - feed balls with pauses for velocity recovery
        // Ball 1
        setIntakeFull();
        sleep(1500);  // Feed All balls

        // 6. Stop everything
        setIntake(false);
        stopper.setPosition(STOPPER_CLOSE);
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);

    }

    private void startFlywheels() {
        // Pre-spin at SAMPLE velocity for intaked ball shots
        outtake1.setVelocity(SAMPLE_VELOCITY);
        outtake2.setVelocity(SAMPLE_VELOCITY);
    }

    private void setTurretToFixedAngle(double angleRadians) {
        // Clamp angle to limits
        double clampedAngle = Math.max(-MAX_TURRET_ANGLE, Math.min(MAX_TURRET_ANGLE, angleRadians));

        // Convert to encoder ticks
        int targetTicks = (int)(clampedAngle * TICKS_PER_RADIAN);

        // Set turret position
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);

        telemetry.addData("Turret Target", "%.1f° (%d ticks)", Math.toDegrees(clampedAngle), targetTicks);
    }

    // ==================== HELPER METHODS ====================

            private static double calculateTurretAngle(double robotX, double robotY, double robotHeading) {
        // Calculate angle from robot position to goal
        double deltaX = BLUE_GOAL_X - robotX;
        double deltaY = BLUE_GOAL_Y - robotY;
        double fieldAngleToGoal = Math.atan2(deltaY, deltaX);

        // Calculate required turret angle (relative to robot)
        double turretAngle = fieldAngleToGoal - robotHeading;

        // Normalize to [-PI, PI]
        while (turretAngle > Math.PI) turretAngle -= 2.0 * Math.PI;
        while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

        return turretAngle;
    }

    private void setIntake(boolean on) {
        intake.setPower(on ? INTAKE_FULL_POWER : 0.0);
    }

    private void setIntakeHold() {
        intake.setPower(INTAKE_HOLD_POWER);  // Low power to hold ball
    }

    private void setIntakeFull() {
        intake.setPower(INTAKE_FULL_POWER);  // Full power for shooting
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
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize motors
        intake = hardwareMap.dcMotor.get("intake");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        stopper = hardwareMap.servo.get("stopper");

        // Motor directions
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Turret setup
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Outtake PIDF
        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);
        outtake2.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);

        // Start with stopper closed
        stopper.setPosition(STOPPER_CLOSE);
    }

    private void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(58.000, 9.000),
                                new Pose(68.860, 22.654),
                                new Pose(44.860, 39.477)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                                        new BezierLine(new Pose(44.860, 39.477), new Pose(12, 40.150))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(19.738, 40.150),
                                new Pose(67.514, 34.318),
                                new Pose(58.000, 15.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(58.000, 15.000),
                                new Pose(58.318, 56.075),
                                new Pose(43.963, 63.477)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                                        new BezierLine(new Pose(43.963, 63.477), new Pose(12, 63.701))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(14.804, 63.701),
                                new Pose(60.336, 62.579),
                                new Pose(57.869, 15.028)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(57.869, 15.028),
                                new Pose(66.168, 68.636),
                                new Pose(43.065, 87.028)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                                        new BezierLine(new Pose(43.065, 87.028), new Pose(22, 87.925))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                                        new BezierCurve(
                                                new Pose(16.598, 87.925),
                                new Pose(64.822, 52.262),
                                new Pose(57.645, 15.028)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }
}