package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source.Constants;

@Autonomous(name="BlueAutoFarTest", group="Autonomous")
public class BlueAutoFarTest extends OpMode {

    // ---------------- Hardware ----------------
    private DcMotor intake, outtake1, outtake2;
    private Servo barrel, pusher;
    private static final double OUTTAKE_POWER = 0.36;

    // Servo positions
    private static final double HOLDER_START = 0.18;
    private static final double PUSHER_RETRACT = 0.03;
    private static final double PUSHER_EXTEND = 0.25;
    private static final double INTAKE_1 = 0.19;
    private static final double INTAKE_2 = 0.35;
    private static final double INTAKE_3 = 0.53;
    private static final double SHOOT_1 = 0.65;
    private static final double SHOOT_2 = 0.43;
    private static final double SHOOT_3 = 0.80;

    // ---------------- Pathing ----------------
    private Follower follower;
    private Paths paths;

    // ---------------- Telemetry ----------------
    private TelemetryManager panelsTelemetry;
    private ElapsedTime timer = new ElapsedTime();

    // ---------------- State Machine ----------------
    private enum State {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE1, DRIVE_TO_INTAKE2, DRIVE_TO_INTAKE3,
        INTAKE1_WAIT, INTAKE2_WAIT, INTAKE3_WAIT,
        RETURN_TO_SHOOT,
        SHOOT_COLLECTED,
        END
    }
    private State state = State.DRIVE_TO_SHOOT;
    private boolean pathStarted = false;
    private int shootIndex = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        barrel = hardwareMap.get(Servo.class, "holder");
        pusher = hardwareMap.get(Servo.class, "pusher");
        barrel.setPosition(HOLDER_START);
        pusher.setPosition(PUSHER_RETRACT);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(50, 8, Math.toRadians(90)));

        paths = new Paths(follower);
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {

            case DRIVE_TO_SHOOT:
                startPath(paths.Path1, State.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootNonBlocking();
                }
                break;

            case DRIVE_TO_INTAKE1:
                startPath(paths.Path2, State.INTAKE1_WAIT);
                startIntake(INTAKE_1);
                break;

            case INTAKE1_WAIT:
                waitIntake(State.DRIVE_TO_INTAKE2);
                break;

            case DRIVE_TO_INTAKE2:
                startPath(paths.Path3, State.INTAKE2_WAIT);
                startIntake(INTAKE_2);
                break;

            case INTAKE2_WAIT:
                waitIntake(State.DRIVE_TO_INTAKE3);
                break;

            case DRIVE_TO_INTAKE3:
                startPath(paths.Path4, State.INTAKE3_WAIT);
                startIntake(INTAKE_3);
                break;

            case INTAKE3_WAIT:
                waitIntake(State.RETURN_TO_SHOOT);
                break;

            case RETURN_TO_SHOOT:
                startPath(paths.Path5, State.SHOOT_COLLECTED);
                break;

            case SHOOT_COLLECTED:
                if (!follower.isBusy()) {
                    shootNonBlocking();
                }
                break;

            case END:
                intake.setPower(0);
                stopShooter();
                panelsTelemetry.debug("Status", "Autonomous Complete");
                break;
        }

        panelsTelemetry.debug("State", state.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // ---------------- Helpers ----------------
    private void startPath(PathChain path, State next) {
        if (!pathStarted) {
            follower.followPath(path, true);
            pathStarted = true;
        }
        if (!follower.isBusy()) {
            pathStarted = false;
            state = next;
            timer.reset();
            shootIndex = 0;
        }
    }

    private void shootNonBlocking() {
        outtake1.setPower(OUTTAKE_POWER);
        outtake2.setPower(OUTTAKE_POWER);

        double[] positions = {SHOOT_1, SHOOT_2, SHOOT_3};

        if (shootIndex < positions.length) {
            double pos = positions[shootIndex];
            if (timer.milliseconds() < 1500) {
                barrel.setPosition(pos);
            } else if (timer.milliseconds() < 3000) {
                pusher.setPosition(PUSHER_EXTEND);
            } else if (timer.milliseconds() < 4500) {
                pusher.setPosition(PUSHER_RETRACT);
            } else {
                shootIndex++;
                timer.reset();
            }
        } else {
            stopShooter();
            shootIndex = 0;
            if (state == State.SHOOT_PRELOAD) state = State.DRIVE_TO_INTAKE1;
            else if (state == State.SHOOT_COLLECTED) state = State.END;
        }
    }

    private void stopShooter() {
        outtake1.setPower(0);
        outtake2.setPower(0);
    }

    private void startIntake(double pos) {
        barrel.setPosition(pos);
        intake.setPower(1.0);
        timer.reset();
    }

    private void waitIntake(State next) {
        if (timer.milliseconds() > 3000) {
            intake.setPower(0);
            timer.reset();
            state = next;
        }
    }

    // ---------------- Paths ----------------
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(50,8), new Pose(50,80)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(50,80), new Pose(50,76)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(50,76), new Pose(47,76)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(47,76), new Pose(44,76)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(44,76), new Pose(41,76)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(41,76), new Pose(50,80)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();
        }
    }

}
