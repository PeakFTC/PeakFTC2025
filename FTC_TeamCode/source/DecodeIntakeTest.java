package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DecodeIntakeTest extends OpMode {

    private DcMotor intake;
    private DcMotor outtake;
    private DcMotor outtake2;

    private DcMotor FLeft;
    private DcMotor BLeft;
    private DcMotor Fright;
    private DcMotor Bright;

    private Servo Intake_barrel;
    private Servo pusher;

    // ✅ CONTINUOUS ROTATION SERVO
    private CRServo helpIn;

    private ElapsedTime time;

    private NormalizedColorSensor colorSensor;

    private enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private int barrelCnt;
    private int outCnt;

    private boolean lastBPressed = false;
    private boolean lastYPressed = false;
    private boolean lastTriggerPressed = false;
    private boolean lastOuttakeTogglePressed = false;

    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastPusherPressed = false;

    private double outtakePower = 0.37;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void init() {
        time = new ElapsedTime();

        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake1");
        outtake2 = hardwareMap.dcMotor.get("outtake2");

        FLeft = hardwareMap.dcMotor.get("FL");
        BLeft = hardwareMap.dcMotor.get("BL");
        Fright = hardwareMap.dcMotor.get("FR");
        Bright = hardwareMap.dcMotor.get("BR");

        Intake_barrel = hardwareMap.servo.get("holder");
        pusher = hardwareMap.servo.get("pusher");

        // ✅ CRServo INIT
        helpIn = hardwareMap.crservo.get("helpIn");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Fright.setDirection(DcMotorSimple.Direction.REVERSE);
        Bright.setDirection(DcMotorSimple.Direction.REVERSE);
        BLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        helpIn.setDirection(DcMotorSimple.Direction.REVERSE);

        // OPTIONAL: reverse CRServo if needed
        // helpIn.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake_barrel.setPosition(0.18);
        barrelCnt = 0;
        pusher.setPosition(0.03);
    }

    @Override
    public void start() {
        time.reset();
    }

    @Override
    public void loop() {
        handleIntake();
        handleOuttakeToggle();
        handleOuttakePowerAdjust();
        moveDriveTrain();
        handleBarrel();
        handlePusher();
        handleBarrelIncrement();
        Out_shoot();

        telemetry.addData("Color detected", getColorString());
        telemetry.addData("Barrel Count", barrelCnt);
        telemetry.addData("Outtake On?", outtakeOn);
        telemetry.addData("Outtake Power", outtakePower);
        telemetry.addData("Intake On?", intakeOn);
        telemetry.addData("HelpIn Power", intakeOn ? 1.0 : 0.0);
        telemetry.update();
    }

    private double ScalePower(double power) {
        return Math.max(-0.8, Math.min(0.8, power));
    }

    private void moveDriveTrain() {
        double forward = -gamepad1.right_stick_y;
        double turn = -gamepad1.right_stick_x;
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

    // ✅ INTAKE + HELPIN LINKED
    private void handleIntake() {
        boolean triggerPressed = gamepad1.right_trigger > 0.5;

        if (triggerPressed && !lastTriggerPressed) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            intake.setPower(1.0);
            helpIn.setPower(1.0); // CRServo ON
        } else {
            intake.setPower(0.0);
            helpIn.setPower(0.0); // CRServo OFF
        }

        lastTriggerPressed = triggerPressed;
    }

    private void handleOuttakeToggle() {
        boolean bPressed = gamepad1.b;

        if (bPressed && !lastOuttakeTogglePressed) {
            outtakeOn = !outtakeOn;
        }

        if (outtakeOn) {
            outtake.setPower(outtakePower);
            outtake2.setPower(outtakePower);
        } else {
            outtake.setPower(0);
            outtake2.setPower(0);
        }

        lastOuttakeTogglePressed = bPressed;
    }

    private void handleOuttakePowerAdjust() {
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;

        if (right && !lastDpadRight) {
            outtakePower = Math.min(outtakePower + 0.05, 1.0);
        }

        if (left && !lastDpadLeft) {
            outtakePower = Math.max(outtakePower - 0.05, 0.0);
        }

        lastDpadRight = right;
        lastDpadLeft = left;

        if (outtakeOn) {
            outtake.setPower(outtakePower);
            outtake2.setPower(outtakePower);
        }
    }

    private void handleBarrel() {
        boolean xPressed = gamepad1.x;

        if (xPressed && !lastBPressed) {
            barrelCnt = (barrelCnt + 1) % 5;

            switch (barrelCnt) {
                case 0: Intake_barrel.setPosition(0.18); break;
                case 1: Intake_barrel.setPosition(0.19); break;
                case 2: Intake_barrel.setPosition(0.35); break;
                case 3: Intake_barrel.setPosition(0.53); break;
                case 4: Intake_barrel.setPosition(0.5994); break;
            }
        }
        lastBPressed = xPressed;
    }

    private void Out_shoot() {
        boolean yPressed = gamepad1.y;

        if (yPressed && !lastYPressed) {
            outCnt = (outCnt + 1) % 3;

            switch (outCnt) {
                case 0: Intake_barrel.setPosition(0.65); break;
                case 1: Intake_barrel.setPosition(0.43); break;
                case 2: Intake_barrel.setPosition(0.8); break;
            }
        }
        lastYPressed = yPressed;
    }

    private boolean pusherActive = false;
    private double pusherStartTime = 0;

    private void handlePusher() {
        boolean aPressed = gamepad1.a;

        if (aPressed && !lastPusherPressed) {
            pusher.setPosition(0.25);
            pusherActive = true;
            pusherStartTime = time.milliseconds();
        }

        if (pusherActive && time.milliseconds() - pusherStartTime > 400) {
            pusher.setPosition(0.03);
            pusherActive = false;
        }

        lastPusherPressed = aPressed;
    }

    private void handleBarrelIncrement() {
        if (gamepad1.dpad_up && !lastDpadUp) {
            Intake_barrel.setPosition(Math.min(Intake_barrel.getPosition() + 0.05, 1.0));
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            Intake_barrel.setPosition(Math.max(Intake_barrel.getPosition() - 0.05, 0.0));
        }
        lastDpadDown = gamepad1.dpad_down;
    }

    private DetectedColor detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red * 100;
        float green = colors.green * 100;
        float blue = colors.blue * 100;

        if (green > red && green > blue && green > 40) return DetectedColor.GREEN;
        if (red > 20 && blue > 20 && (red + blue) > 2 * green) return DetectedColor.PURPLE;

        return DetectedColor.UNKNOWN;
    }

    private String getColorString() {
        return detectColor().name();
    }
}
