package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source.Constants;

@Autonomous(name = "Test Auto", group = "Examples")
public class testAuto extends LinearOpMode{
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    private Follower follower;

    void buildPath(){

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(56.000, 35.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 35.500),

                                //new Pose(23.000, 35.500)
                                new Pose(30, 35.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                //new Pose(23.000, 35.500),
                                new Pose(30.000, 35.500),

                                new Pose(56.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();
    }
    void updateTele(){
        telemetry.addData("XPos =%d" ,follower.getPose().getX());
        telemetry.addData("Ypos= %d ", follower.getPose().getY());
        telemetry.addData("heading = %d", follower.getHeading());
        telemetry.update();

    }
    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        buildPath();

        follower.setStartingPose(  new Pose(56.000, 8.000,Math.toRadians(90)));
        updateTele();
        waitForStart();
        follower.followPath(Path1);
        follower.update();

        while (follower.isBusy())
        {
            follower.update();
            updateTele();
        }
        updateTele();
        sleep(2000);
         follower.followPath(Path2);
         follower.update();
        while (follower.isBusy())
        {
          follower.update();
          updateTele();
        }
        sleep(2000);
        follower.followPath(Path3);
        follower.update();
        while (follower.isBusy())
        {
            follower.update();
            updateTele();
        }
        sleep(2000);
    }


}