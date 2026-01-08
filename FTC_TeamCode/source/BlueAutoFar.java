package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pedro Pathing Autonomous OpMode
 *
 * This autonomous program implements a complex state machine to:
 * 1. Shoot preloaded balls at the basket
 * 2. Intake artifacts from 3 different levels
 * 3. Return to shooting position and score
 * 4. Repeat for multiple scoring cycles
 * 5. Park at the end
 *
 * Features:
 * - Multi-level artifact collection (Level 1, 2, 3)
 * - Non-blocking timer system
 * - Sequential barrel rotation for intake/outtake
 * - Pedro Pathing for precise navigation
 * - Panels telemetry integration
 *
 * @author FTC Team
 * @version 1.0
 */
@Autonomous(name = "BlueAutoFar", group = "Autonomous")
@Configurable // Panels
public class BlueAutoFar extends OpMode {
    // ============================================
    // HARDWARE COMPONENTS
    // ============================================

    /** Intake motor - pulls balls into the robot */
    private DcMotor intake;

    /** Primary outtake motor - shoots balls into basket */
    private DcMotor outtake;

    /** Secondary outtake motor - works with outtake for shooting */
    private DcMotor outtake2;

    /** Power level for outtake motors (0-1.0) */
    private double outtakePower = 0.41;

    /** Servo that rotates the intake barrel to different ball positions */
    private Servo Intake_barrel;

    /** Servo that pushes balls from barrel into outtake mechanism */
    private Servo pusher;

    /** Counter to track current barrel position during intake (0-4) */
    private int barrelCnt;

    /** Counter to track current barrel position during outtake (0-2) */
    private int outCnt;

    // ============================================
    // TIMING & TELEMETRY
    // ============================================

    /** Non-blocking delay system - tracks if delay is active */
    private boolean delayActive = false;

    /** End time for non-blocking delay in milliseconds */
    private double delayEndTime = 0;

    /** Global timer for non-blocking delays */
    private ElapsedTime globalTimer = new ElapsedTime();

    /** Timer for state machine actions and transitions */
    private ElapsedTime actionTimer = new ElapsedTime();

    /** Panels telemetry manager for enhanced dashboard display */
    private TelemetryManager panelsTelemetry;

    // ============================================
    // PEDRO PATHING & STATE MACHINE
    // ============================================

    /** Pedro Pathing follower for autonomous navigation */
    public Follower follower;

    /** Current state value for telemetry (ordinal of enum) */
    private int pathState;

    /** Container for all autonomous paths */
    private Paths paths;

    /**
     * Resets the action timer to 0.
     * Used to start timing for state transitions or actions.
     */
    public void startTimer() {
        actionTimer.reset();
    }

    /**
     * Checks if the action timer has exceeded the specified delay.
     * Note: Currently hardcoded to 2 seconds, delay_s parameter is not used.
     *
     * @param delay_s The desired delay in seconds (currently ignored)
     * @return true if timer has exceeded 2 seconds, false otherwise
     */
    public boolean isTimerExpired(double delay_s) {
        boolean ex = false;
        if (actionTimer.seconds() >= 2) {
            ex = true;
        }
        return ex;
    }

    /**
     * Initializes hardware, Pedro Pathing, and autonomous paths.
     * Called once when the INIT button is pressed.
     *
     * Setup sequence:
     * 1. Initialize motors and servos from hardware map
     * 2. Configure motor directions
     * 3. Set servos to starting positions
     * 4. Initialize telemetry and Pedro Pathing
     * 5. Set starting robot pose (56, 8) facing 90°
     * 6. Build all autonomous paths
     */
    @Override
    public void init() {
        // Initialize motors from hardware configuration
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake1");
        outtake2 = hardwareMap.dcMotor.get("outtake2");

        // Configure motor directions for proper operation
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize servos from hardware configuration
        Intake_barrel = hardwareMap.servo.get("holder");
        pusher = hardwareMap.servo.get("pusher");

        // Set servos to starting/safe positions
        Intake_barrel.setPosition(0.18);  // Starting barrel position
        pusher.setPosition(0.03);          // Pusher retracted position

        // Initialize Panels telemetry for enhanced dashboard
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose: X=56, Y=8, Heading=90° (facing up the field)
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        // Build all autonomous path chains
        paths = new Paths(follower);

        // Display initialization status
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    /**
     * Starts a non-blocking delay timer.
     * Sets the delay end time based on current globalTimer value.
     *
     * @param ms Delay duration in milliseconds
     */
    private void startDelay(double ms) {
        delayActive = true;
        delayEndTime = globalTimer.milliseconds() + ms;
    }

    /**
     * BLOCKING delay function - halts all code execution.
     * WARNING: This is a blocking wait and should be avoided in loop().
     * Used only in autonomous action sequences like shootThreeBalls().
     *
     * @param ms Time to wait in milliseconds
     */
    private void waitMs(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // Busy wait - blocks execution
        }
    }

    /**
     * Shoots three balls sequentially into the basket.
     * This is a BLOCKING function that uses waitMs() delays.
     *
     * Sequence for each ball:
     * 1. Rotate barrel to ball position (0.65, 0.43, or 0.80)
     * 2. Wait 250ms for barrel to settle
     * 3. Push ball into shooter with pusher servo (0.25)
     * 4. Wait 200ms for ball to enter shooter
     * 5. Retract pusher (0.03)
     * 6. Wait 200ms before next ball
     *
     * Total execution time: ~1950ms (blocking)
     */
    private void shootThreeBalls() {
        // Spin up shooter wheels to target velocity
        outtake.setPower(outtakePower);
        outtake2.setPower(outtakePower);

        // Ball 1: Rotate barrel to first ball position
        Intake_barrel.setPosition(0.65);
        waitMs(250);  // Allow barrel rotation
        pusher.setPosition(0.25);  // Push ball into shooter
        waitMs(200);  // Allow ball to enter
        pusher.setPosition(0.03);  // Retract pusher
        waitMs(200);  // Settling time

        // Ball 2: Rotate barrel to second ball position
        Intake_barrel.setPosition(0.43);
        waitMs(250);
        pusher.setPosition(0.25);
        waitMs(200);
        pusher.setPosition(0.03);
        waitMs(200);

        // Ball 3: Rotate barrel to third ball position
        Intake_barrel.setPosition(0.80);
        waitMs(250);
        pusher.setPosition(0.25);
        waitMs(200);
        pusher.setPosition(0.03);
        waitMs(200);

        // Stop shooter wheels after all balls are shot
        outtake.setPower(0);
        outtake2.setPower(0);
    }

    /**
     * Intakes a single ball from the field into a specific barrel position.
     * This is a BLOCKING function.
     *
     * Barrel positions:
     * - Index 1: Position 0.19 (first ball slot)
     * - Index 2: Position 0.35 (second ball slot)
     * - Index 3: Position 0.53 (third ball slot)
     *
     * @param index Ball position number (1, 2, or 3)
     *
     * Sequence:
     * 1. Spin intake motor at full power
     * 2. Rotate barrel to target position
     * 3. Wait 400ms for ball to be collected
     * 4. Stop intake motor
     */
    private void intakeBall(int index) {
        // Start intake roller
        intake.setPower(1.0);

        // Position barrel based on which ball slot to fill
        if (index == 1) Intake_barrel.setPosition(0.19);
        if (index == 2) Intake_barrel.setPosition(0.35);
        if (index == 3) Intake_barrel.setPosition(0.53);

        // Allow time for ball collection
        waitMs(400);

        // Stop intake
        intake.setPower(0);
    }


    /**
     * Starts the launcher (outtake) motors.
     * Does NOT stop them - must be manually stopped later.
     * Used for pre-spinning shooter before firing sequence.
     */
    private void startlaunch() {
        outtake.setPower(outtakePower);
        outtake2.setPower(outtakePower);
    }

    /**
     * Cycles the intake barrel through positions for ball collection.
     * Increments barrel counter and wraps around after position 4.
     *
     * Positions:
     * 0: 0.18 - Starting/neutral position
     * 1: 0.19 - First ball intake slot
     * 2: 0.35 - Second ball intake slot
     * 3: 0.53 - Third ball intake slot
     * 4: 0.5994 - Fourth position (alternate)
     */
    private void IntakeBarrel() {
        barrelCnt++;
        if (barrelCnt > 4) barrelCnt = 0;

        switch (barrelCnt) {
            case 0:
                Intake_barrel.setPosition(0.18);
                break;
            case 1:
                Intake_barrel.setPosition(0.19);
                break;
            case 2:
                Intake_barrel.setPosition(0.35);
                break;
            case 3:
                Intake_barrel.setPosition(0.53);
                break;
            case 4:
                Intake_barrel.setPosition(0.5994);
                break;
        }
    }

    /**
     * Cycles the barrel through shooting positions.
     * Increments outtake counter and wraps around after position 2.
     *
     * Positions:
     * 0: 0.65 - First ball shooting position
     * 1: 0.43 - Second ball shooting position
     * 2: 0.8  - Third ball shooting position
     */
    private void OuttakeBarrel() {
        outCnt++;
        if (outCnt > 2) outCnt = 0;

        switch (outCnt) {
            case 0:
                Intake_barrel.setPosition(0.65);
                break;
            case 1:
                Intake_barrel.setPosition(0.43);
                break;
            case 2:
                Intake_barrel.setPosition(0.8);
                break;
        }
    }

    /**
     * Pushes a ball from the barrel into the shooter mechanism.
     * NOTE: Incomplete implementation - missing delay between positions.
     * Should add a timer to ensure pusher extends then retracts.
     */
    private void handlePusher() {
        pusher.setPosition(0.25); // Shoot Position - extended
        // TODO: Add 200ms delay here
        pusher.setPosition(0.03); // Start Position - retracted
    }


    /**
     * Main loop function - called repeatedly during autonomous.
     *
     * Responsibilities:
     * 1. Update Pedro Pathing follower (processes path following)
     * 2. Update state machine and get current state
     * 3. Display telemetry data to driver station and dashboard
     *
     * Telemetry includes:
     * - Current state machine state
     * - Robot X, Y position on field
     * - Robot heading angle
     */
    @Override
    public void loop() {
        // Update path follower - MUST be called every loop
        follower.update();

        // Execute state machine and get current state value
        pathState = autonomousPathUpdate();

        // Send telemetry to Panels dashboard and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /**
     * Inner class containing all autonomous path definitions.
     *
     * Paths are organized by scoring round and purpose:
     *
     * Round 1:
     * - Shoot_1_path: Drive from start to first shooting position
     * - Level_1_Intake_path + artifacts 1-3: Collect balls from level 1
     *
     * Round 2:
     * - Shoot_2_path: Return to shooting position
     * - Level_2_Intake_path + artifacts 1-3: Collect balls from level 2
     *
     * Round 3:
     * - Shoot_3_path: Return to shooting position
     * - Level_3_Intake_path + artifacts 1-3: Collect balls from level 3
     *
     * Round 4:
     * - Shoot_4_path: Final shooting position
     * - park_path: Drive to parking zone
     *
     * All paths use:
     * - BezierLine for straight segments
     * - BezierCurve for smooth turns (with control points)
     * - TangentHeadingInterpolation for natural heading along path
     * - setReversed() for driving backwards
     */
    public static class Paths {

        public PathChain Shoot_1_path;
        public PathChain Level_1_Intake_path;
        public PathChain Level_1_Intake_path_artifact1;
        public PathChain Level_1_Intake_path_artifact2;
        public PathChain Level_1_Intake_path_artifact3;


        public PathChain Level_2_Intake_path;
        public PathChain Level_2_Intake_path_artifact1;
        public PathChain Level_2_Intake_path_artifact2;
        public PathChain Level_2_Intake_path_artifact3;

        public PathChain Level_3_Intake_path;
        public PathChain Level_3_Intake_path_artifact1;
        public PathChain Level_3_Intake_path_artifact2;
        public PathChain Level_3_Intake_path_artifact3;

        public PathChain Shoot_2_path;
        public PathChain Shoot_3_path;
        public PathChain Shoot_4_path;
        public PathChain park_path;


        public Paths(Follower follower) {
            Shoot_1_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(61.907, 46.206))
                    )
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.907, 46.206),
                                    new Pose(77.159, 63.028),
                                    new Pose(60.112, 82.991)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Level_1_Intake_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.112, 82.991),
                                    new Pose(47.776, 92.187),
                                    new Pose(38.131, 84.112)
                            )
                    )
                    .setReversed()
                    .setTangentHeadingInterpolation()
                    .build();
            Level_1_Intake_path_artifact1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(38.131, 84.112), new Pose(32.299, 83.664))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            Level_1_Intake_path_artifact2= follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.299, 83.664), new Pose(24.897, 83.888))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            Level_1_Intake_path_artifact3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.897, 83.888), new Pose(19.738, 84.112))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            Shoot_2_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.738, 84.112), new Pose(65.271, 69.533))
                    )

                    .addPath(
                            new BezierCurve(
                                    new Pose(65.271, 69.533),
                                    new Pose(71.776, 68.411),
                                    new Pose(60.112, 83.215)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Level_2_Intake_path = follower
                    .pathBuilder()
                    .addPath( new BezierCurve(
                                    new Pose(60.112, 83.215),
                                    new Pose(62.355, 61.458),
                                    new Pose(45.981, 60.112)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_2_Intake_path_artifact1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.981, 60.112), new Pose(33.421, 60.336))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_2_Intake_path_artifact2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.421, 60.336), new Pose(27.140, 60.336))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_2_Intake_path_artifact3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(27.140, 60.336), new Pose(20.860, 60.112))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Shoot_3_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.860, 60.112), new Pose(64.598, 62.131))
                    )
                    .setReversed()
                    .addPath( new BezierCurve(
                            new Pose(64.598, 62.131),
                            new Pose(74.019, 66.168),
                            new Pose(60.112, 82.318)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Level_3_Intake_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.112, 82.318),
                                    new Pose(69.000, 46.000),
                                    new Pose(48.000, 35.215)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_3_Intake_path_artifact1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 35.215), new Pose(33.645, 35.439))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_3_Intake_path_artifact2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.645, 35.439), new Pose(26.243, 36.112))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Level_3_Intake_path_artifact3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(26.243, 36.112), new Pose(21.757, 36.112))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            Shoot_4_path = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.757, 36.112), new Pose(62.355, 54.505))
                    )

                    .addPath(new BezierCurve(
                                    new Pose(62.355, 54.505),
                                    new Pose(76.262, 65.271),
                                    new Pose(60.561, 82.766)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            park_path = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(60.561, 82.766), new Pose(31.850, 72.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }


    /**
     * Main autonomous state machine enumeration.
     * Defines the sequential states for the complete autonomous routine.
     *
     * State Flow:
     * START → SHOOT_FIRST → DRIVE_L1 → INTAKE_2ND → DRIVE_FOR_2ND → SHOOT_2ND
     *      → DRIVE_L2 → INTAKE_3RD → DRIVE_FOR_3RD → SHOOT_3RD
     *      → DRIVE_L3 → INTAKE_4TH → DRIVE_FOR_4TH → SHOOT_4TH
     *      → PARK → END
     *
     * Each "ROUND" consists of:
     * 1. DRIVE - Navigate to intake zone
     * 2. INTAKE - Collect 3 artifacts sequentially
     * 3. DRIVE_FOR - Return to shooting position
     * 4. SHOOT - Score all collected balls
     */
    enum AutonStateMachine {
        STATE_MACHINE_START,                          // Initial state
        STATE_MACHINE_SHOOT_FIRST_ROUND,              // Score preloaded balls
        STATE_MACHINE_AUTON_DRIVE_LEVEL1,             // Drive to level 1 intake
        STATE_MACHINE_INTAKE_SECOND_ROUND,            // Collect 3 balls from level 1
        STATE_MACHINE_AUTON_DRIVE_FOR_SECOND_ROUND,   // Return to shooting position
        STATE_MACHINE_SHOOT_SECOND_ROUND,             // Score round 2 balls
        STATE_MACHINE_AUTON_DRIVE_LEVEL2,             // Drive to level 2 intake
        STATE_MACHINE_INTAKE_THIRD_ROUND,             // Collect 3 balls from level 2
        STATE_MACHINE_AUTON_DRIVE_FOR_THIRD_ROUND,    // Return to shooting position
        STATE_MACHINE_SHOOT_THIRD_ROUND,              // Score round 3 balls
        STATE_MACHINE_AUTON_DRIVE_LEVEL3,             // Drive to level 3 intake
        STATE_MACHINE_INTAKE_FOURTH_ROUND,            // Collect 3 balls from level 3
        STATE_MACHINE_AUTON_DRIVE_FOR_FOURTH_ROUND,   // Return to shooting position
        STATE_MACHINE_SHOOT_FOURTH_ROUND,             // Score round 4 balls
        STATE_MACHINE_AUTON_PARK,                     // Drive to parking zone
        STATE_MACHINE_END                             // Autonomous complete
    }

    /**
     * Sub-state machine for artifact collection.
     * Tracks which of the 3 artifacts is currently being collected.
     * Resets after all 3 artifacts are collected in each round.
     */
    enum ArtifactPosition {
        ARTIFACT_ONE,   // First ball position
        ARTIFACT_TWO,   // Second ball position
        ARTIFACT_THREE, // Third ball position
        ARTIFACT_END    // All balls collected, ready to advance
    }
    /** Current state of the main autonomous state machine */
    private AutonStateMachine stateMachine = AutonStateMachine.STATE_MACHINE_START;

    /** Current artifact being collected (1st, 2nd, or 3rd) */
    private ArtifactPosition artifactPosition = ArtifactPosition.ARTIFACT_ONE;

    /**
     * Handles artifact intake based on current artifact position.
     * Sub-state machine that cycles through 3 artifact positions.
     *
     * Flow:
     * ARTIFACT_ONE → intake ball 1 → advance to ARTIFACT_TWO
     * ARTIFACT_TWO → intake ball 2 → advance to ARTIFACT_THREE
     * ARTIFACT_THREE → intake ball 3 → advance to ARTIFACT_END
     *                                  → transition main state to DRIVE_FOR_SECOND_ROUND
     *
     * After all 3 artifacts collected:
     * - Sets artifactPosition to ARTIFACT_END
     * - Advances main state machine to return to shooting position
     */
    private void intakeArtifact() {
        switch (artifactPosition) {
            case ARTIFACT_ONE:
                intakeBall(1);  // Collect first ball
                artifactPosition = ArtifactPosition.ARTIFACT_TWO;
                break;

            case ARTIFACT_TWO:
                intakeBall(2);  // Collect second ball
                artifactPosition = ArtifactPosition.ARTIFACT_THREE;
                break;

            case ARTIFACT_THREE:
                intakeBall(3);  // Collect third ball
                artifactPosition = ArtifactPosition.ARTIFACT_END;

                break;

            default:
                break;
        }
    }
    /**
     * Main state machine update function.
     * Called every loop to process current state and determine transitions.
     *
     * State Machine Pattern:
     * Each state checks if the follower is busy (!follower.isBusy())
     * When path completes:
     * - Execute associated action (shoot, intake, etc.)
     * - Transition to next state
     *
     * State Categories:
     * - START: Initialize first path
     * - SHOOT states: Score balls (blocking operation)
     * - DRIVE states: Start navigation path
     * - INTAKE states: Collect artifacts (sub-state machine)
     * - PARK: Final positioning
     * - END: Autonomous complete
     *
     * @return Current state ordinal value for telemetry
     */
    public int autonomousPathUpdate() {

        switch (stateMachine) {

            case STATE_MACHINE_START:
                follower.followPath(paths.Shoot_1_path);
                stateMachine = AutonStateMachine.STATE_MACHINE_SHOOT_FIRST_ROUND;
                break;

            case STATE_MACHINE_SHOOT_FIRST_ROUND:
                if (!follower.isBusy()) {
                    shootThreeBalls();  // FIRST ROUND
                    stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_LEVEL1;
                }
                break;

            case STATE_MACHINE_AUTON_DRIVE_LEVEL1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Level_1_Intake_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_INTAKE_SECOND_ROUND;
                    waitMs(1000);
                }
                break;

            case STATE_MACHINE_INTAKE_SECOND_ROUND:
                if (!follower.isBusy()) {
                    intakeArtifact();
                    if (artifactPosition == ArtifactPosition.ARTIFACT_END) {
                        artifactPosition = ArtifactPosition.ARTIFACT_ONE;
                        stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_FOR_SECOND_ROUND;
                        waitMs(1000);
                    }

                }
                break;

            case STATE_MACHINE_AUTON_DRIVE_FOR_SECOND_ROUND:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot_2_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_SHOOT_SECOND_ROUND;
                    waitMs(1000);
                    stateMachine=AutonStateMachine.STATE_MACHINE_AUTON_PARK;

                }
                break;

            case STATE_MACHINE_SHOOT_SECOND_ROUND:
                if (!follower.isBusy()) {
                    shootThreeBalls();  // SECOND ROUND
                    stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_LEVEL2;
                    stateMachine = AutonStateMachine.STATE_MACHINE_END;
                }
                break;

            // -------------------------
            // PATH 5 → Intake 3rd ball
            // -------------------------
            case STATE_MACHINE_AUTON_DRIVE_LEVEL2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Level_2_Intake_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_INTAKE_THIRD_ROUND;
                }
                break;

            case STATE_MACHINE_INTAKE_THIRD_ROUND:
                if (!follower.isBusy()) {
                    intakeArtifact();
                    if (artifactPosition == ArtifactPosition.ARTIFACT_END) {
                        artifactPosition = ArtifactPosition.ARTIFACT_ONE;
                        stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_FOR_THIRD_ROUND;
                    }

                }
                // -------------------------
                // PATH 6 → Shoot 3 balls (2nd round)
                // -------------------------
            case STATE_MACHINE_AUTON_DRIVE_FOR_THIRD_ROUND:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot_3_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_SHOOT_THIRD_ROUND;
                }

                break;

            case STATE_MACHINE_SHOOT_THIRD_ROUND:
                if (!follower.isBusy()) {
                    shootThreeBalls();  // SECOND ROUND
                    stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_LEVEL3;
                }
                break;

            // -------------------------
            // PATH 7 → Drivetrain only
            // -------------------------
            case STATE_MACHINE_AUTON_DRIVE_LEVEL3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Level_3_Intake_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_INTAKE_FOURTH_ROUND;
                }
                break;


            case STATE_MACHINE_INTAKE_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    intakeArtifact();
                    if (artifactPosition == ArtifactPosition.ARTIFACT_END) {
                        // NOTHING EXTRA
                        artifactPosition = ArtifactPosition.ARTIFACT_ONE;
                        stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_DRIVE_FOR_FOURTH_ROUND;
                    }

                }
                break;

            // -------------------------
            // PATH 8 → Intake 1 ball
            // -------------------------
            case STATE_MACHINE_AUTON_DRIVE_FOR_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot_4_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_SHOOT_FOURTH_ROUND;
                }
                break;


            case STATE_MACHINE_SHOOT_FOURTH_ROUND:
                if (!follower.isBusy()) {
                    shootThreeBalls();  // FOURTH ROUND
                    stateMachine = AutonStateMachine.STATE_MACHINE_AUTON_PARK;
                }
                break;

            case STATE_MACHINE_AUTON_PARK:
                if (!follower.isBusy()) {
                    follower.followPath(paths.park_path);
                    stateMachine = AutonStateMachine.STATE_MACHINE_END;
                }
                break;

            case STATE_MACHINE_END:

                break;
        }

        return stateMachine.ordinal();
    }
}