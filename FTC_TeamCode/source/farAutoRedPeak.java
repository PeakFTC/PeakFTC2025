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

@Autonomous(name = " Far Red Peak Auto", group = "FAR Auto")
public class farAutoRedPeak extends LinearOpMode{


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
    public  PathChain Path10;
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

        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 8.000),
                                new Pose(93.743, 37.701),
                                new Pose(103.000, 40.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.000, 40.000),

                                new Pose(125.000, 40.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.000, 40.000),
                                new Pose(97.075, 42.664),
                                new Pose(80.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(80.500, 15.000),
                                new Pose(91.890, 60.047),
                                new Pose(103.000, 64.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.000, 64.000),

                                new Pose(128.000, 64.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.000, 64.000),
                                new Pose(96.708, 62.930),
                                new Pose(80.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(80.500, 15.000),
                                new Pose(87.507, 74.430),
                                new Pose(103.000, 87.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.000, 87.000),

                                new Pose(125.766, 87.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.766, 87.000),
                                new Pose(92.797, 87.234),
                                new Pose(80.500, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.500, 15.000),

                                new Pose(80.500, 42.000)
                        )
                ).setTangentHeadingInterpolation()

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
        follower.followPath(Path10);
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