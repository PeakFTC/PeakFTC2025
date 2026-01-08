package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Turret Auto Aim - BLUE Goal Pinpoint Final", group = "TeleOp")
public class TeleOp_BlueGoalTurret_PinpointFinal extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx turretMotor;
    private GoBildaPinpointDriver pinpoint;

    /* ===================== FIELD CONSTANTS ===================== */
    static final double BLUE_GOAL_X = 0;    // inches
    static final double BLUE_GOAL_Y = 144;  // inches

    /* ===================== TURRET CONSTANTS ===================== */
    static final double MAX_TURRET_ANGLE = Math.toRadians(90); // ±90°
    static final double TICKS_PER_RADIAN = 428;
    static final double kP_TURRET = 0.85;
    static final double MAX_TURRET_POWER = 0.6;

    /* ===================== ROBOT POSE ===================== */
    double robotX = 56;
    double robotY = 8;
    double robotHeading = Math.toRadians(90);

    /* ===================== CONTROL ===================== */
    boolean turretEnabled = true;  // Auto-enabled now that encoder works

    @Override
    public void init() {
        // Turret motor - NOW WITH WORKING ENCODER
        turretMotor = hardwareMap.get(DcMotorEx.class, "FL");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Pinpoint driver
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.RADIANS, robotHeading));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Turret Encoder", turretMotor.getCurrentPosition());
        telemetry.addData("Info", "Turret will auto-aim at goal");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        robotX = pos.getX(DistanceUnit.INCH);
        robotY = pos.getY(DistanceUnit.INCH);
        robotHeading = pos.getHeading(AngleUnit.RADIANS);

        // Manual control
        if (gamepad1.a) {
            turretEnabled = true;
        }
        if (gamepad1.b) {
            turretEnabled = false;
            turretMotor.setPower(0);
        }

        // Aim turret if enabled
        if (turretEnabled) {
            aimTurretAtBlueGoal(robotX, robotY, robotHeading);
        } else {
            turretMotor.setPower(0);
        }

        // Telemetry
        telemetry.addData("=== ENCODER STATUS ===", "");
        telemetry.addData("Turret Encoder", turretMotor.getCurrentPosition());
        telemetry.addData("Encoder Working?", "If this changes when turret moves = YES");

        telemetry.addData("=== ROBOT POSITION ===", "");
        telemetry.addData("X", "%.1f in", robotX);
        telemetry.addData("Y", "%.1f in", robotY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));

        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Enabled", turretEnabled ? "YES" : "NO");
        telemetry.addData("Power", "%.2f", turretMotor.getPower());

        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Enable auto-aim");
        telemetry.addData("B", "Disable / Emergency stop");

        telemetry.update();
    }

    /* ===================== TURRET AIM LOGIC ===================== */
    void aimTurretAtBlueGoal(double x, double y, double heading) {
        // Field angle to goal
        double angleToGoal = Math.atan2(
                BLUE_GOAL_Y - y,
                BLUE_GOAL_X - x
        );

        // Robot-relative turret angle
        double desiredTurretAngle = AngleUnit.normalizeRadians(angleToGoal - heading);

        // Clamp to physical limits
        desiredTurretAngle = Range.clip(
                desiredTurretAngle,
                -MAX_TURRET_ANGLE,
                MAX_TURRET_ANGLE
        );

        // Get current turret angle from encoder
        double currentTurretAngle = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN;

        // Compute error
        double error = AngleUnit.normalizeRadians(desiredTurretAngle - currentTurretAngle);

        // Apply proportional control
        double power = error * kP_TURRET;

        // Clip power
        power = Range.clip(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        // Deadband to prevent jitter
        if (Math.abs(error) < Math.toRadians(2)) {
            power = 0;
        }

        turretMotor.setPower(power);

        // Additional telemetry
        telemetry.addData("Desired Angle", "%.1f°", Math.toDegrees(desiredTurretAngle));
        telemetry.addData("Current Angle", "%.1f°", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Error", "%.1f°", Math.toDegrees(error));
        telemetry.addData("=== DEBUGGING ===", "");
        telemetry.addData("Robot thinks it's at", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Robot thinks heading is", "%.1f°", Math.toDegrees(robotHeading));
        telemetry.addData("Goal is at", "X: %.1f, Y: %.1f", BLUE_GOAL_X, BLUE_GOAL_Y);
        telemetry.addData("Angle to goal (field)", "%.1f°", Math.toDegrees(angleToGoal));
        telemetry.addData("Desired turret angle", "%.1f°", Math.toDegrees(desiredTurretAngle));
    }
}