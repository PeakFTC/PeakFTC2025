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

@Autonomous(name = " Far Blue Peak Auto", group = "FAR Auto")
public class farAutoBluePeak extends LinearOpMode{


    private DcMotor intake;
    private DcMotorEx outtake;
    private DcMotorEx outtake2;

    private Servo stopper;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    private Follower follower;

    // ==================== SERVO CONSTANTS ====================
    static final double STOPPER_OPEN = 0.22;
    static final double STOPPER_CLOSE = 0.33;

    // ==================== PIDF COEFFICIENTS ====================
    private static final double kP_VELOCITY = 12.0;
    private static final double kI_VELOCITY = 1.8;
    private static final double kD_VELOCITY = 1.8;
    private static final double kF_VELOCITY = 13.2;

    private double targetVelocity = 1276;

    void buildPath(){

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
                        new BezierLine(new Pose(44.860, 39.477), new Pose(19.738, 40.150))
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
                        new BezierLine(new Pose(43.963, 63.477), new Pose(14.804, 63.701))
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
                        new BezierLine(new Pose(43.065, 87.028), new Pose(16.598, 87.925))
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
   private void updateTele(){
        telemetry.addData("XPos =%d" ,follower.getPose().getX());
        telemetry.addData("Ypos= %d ", follower.getPose().getY());
        telemetry.addData("heading = %d", follower.getHeading());
        telemetry.update();

    }

    private void prepareOuttake(){
        outtake.setVelocity(targetVelocity);
        outtake2.setVelocity(targetVelocity);
    }
    private void stopOuttake()
    {
        outtake.setVelocity(0);
        outtake2.setVelocity(0);
    }
    private void shootTheBall() {
        stopper.setPosition(STOPPER_OPEN);
        intake.setPower(1);
        sleep(2000);
        intake.setPower(0);
        stopOuttake();
        stopper.setPosition(STOPPER_CLOSE);
    }

    void startIntake(){
        intake.setPower(1);
    }
    private void stopIntake() {

        intake.setPower(0);
    }

    private void initHW(){
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        stopper = hardwareMap.servo.get("stopper");
        stopper.setPosition(STOPPER_CLOSE);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        // ==================== OUTTAKE SETUP ====================
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);
        outtake2.setVelocityPIDFCoefficients(kP_VELOCITY, kI_VELOCITY, kD_VELOCITY, kF_VELOCITY);

    }
    private  void runAutoCode(){
        prepareOuttake();
        sleep(2000);
        shootTheBall();
        follower.followPath(Path1);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        startIntake();
        updateTele();

        follower.followPath(Path2);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);
        stopIntake();

        follower.followPath(Path3);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            prepareOuttake();
            updateTele();
        }
        shootTheBall();
        //6 ball shoot
        follower.followPath(Path4);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);

        follower.followPath(Path5);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);
        follower.followPath(Path6);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);

        follower.followPath(Path7);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);

        follower.followPath(Path8);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);

        follower.followPath(Path9);
        follower.update();
        while (follower.isBusy()) {
            follower.update();
            updateTele();
        }
        sleep(1000);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        buildPath();
        initHW();

        follower.setStartingPose(  new Pose(56.000, 8.000,Math.toRadians(90)));
        updateTele();

        waitForStart();
        runAutoCode();

    }



}