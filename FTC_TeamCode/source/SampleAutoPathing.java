package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source.Constants;


@TeleOp
public class SampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE < MOVEMENT STATE
        // SHOOT < ATTEMPT TO SHOOT ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    PathState pathstate;

    private final Pose startPose = new Pose(56,8, Math.toRadians(90));
    private final Pose shootPose = new Pose(56,87, Math.toRadians(135));

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        // put in coordinates for starting Pose > ending Pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

    }


    public void statePathUpdate() {
        switch(pathstate) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset's timer and makes new state
                break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter
                    telemetry.addLine("Done Path 1");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;

        }

    }

    public void setPathState(PathState newState) {
        pathstate = newState;
        pathTimer.resetTimer();

    }




    @Override
    public void init() {
        pathstate = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add any other init mechanism : Reset Set Position for Barrel and Pusher

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathstate);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();



        telemetry.addData("path state", pathstate.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }
}
