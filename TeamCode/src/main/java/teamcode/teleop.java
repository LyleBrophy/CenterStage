package teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

@TeleOp (name = "teleop")
public class teleop extends LinearOpMode{
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBR;
    public DcMotor motorBL;
//    public DcMotor[] motors = new DcMotor[4];
    DcMotor body;
    CRServo wrist, rightHanger, leftHanger, grabbie;
    CRServo launcher;

    public double speedMode = 1;

    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBR");
        body = hardwareMap.get(DcMotor.class, "body");
//        arm = hardwareMap.get(CRServo.class, "arm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        rightHanger = hardwareMap.get(CRServo.class, "rightHanger");
        leftHanger = hardwareMap.get(CRServo.class, "leftHanger");
        launcher = hardwareMap.get(CRServo.class, "launcher");
        grabbie = hardwareMap.get(CRServo.class, "grabbie");

        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = forward + right + turn;
            double leftBackPower = forward - right + turn;
            double rightFrontPower = forward - right - turn;
            double rightBackPower = forward + right - turn;
            double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
            boolean needToScale = false;
            for (double power : powers) {
                if (Math.abs(power) > 1) {
                    needToScale = true;
                    break;
                }
            }
            if (needToScale) {
                double greatest = 0;
                for (double power : powers) {
                    if (Math.abs(power) > greatest) {
                        greatest = Math.abs(power);
                    }
                }
                leftFrontPower /= greatest;
                leftBackPower /= greatest;
                rightFrontPower /= greatest;
                rightBackPower /= greatest;
            }
            boolean stop = true;
            for (double power : powers) {
                double stopBuffer = 0;
                if (Math.abs(power) > stopBuffer) {
                    stop = false;
                    break;
                }
            }
            if (stop) {
                leftFrontPower = 0;
                leftBackPower = 0;
                rightFrontPower = 0;
                rightBackPower = 0;
            }
            motorFL.setPower(leftFrontPower);
            motorBL.setPower(leftBackPower);
            motorFR.setPower(rightFrontPower);
            motorBR.setPower(rightBackPower);
        }
        if (gamepad1.left_trigger > 0.1) {
            leftHanger.setPower(1);
            rightHanger.setPower(-1);
        } else if (gamepad1.right_trigger > 0.1) {
            leftHanger.setPower(-1);
            rightHanger.setPower(1);
        }
        if (gamepad1.left_bumper) {
            body.setPower(1);
        } else if (gamepad1.right_bumper) {
            body.setPower(-1);
        } else if (gamepad1.left_bumper == false && gamepad1.right_bumper == false) {
            body.setPower(0);
        }
//        if (gamepad1.dpad_up) {
//            arm.setPower(1);
//        } else if (gamepad1.dpad_down) {
//            arm.setPower(-1);
//        } else if (gamepad1.dpad_up == false && gamepad1.dpad_down == false) {
//            arm.setPower(0);
//        }
        if (gamepad1.x) {
            wrist.setPower(1);
        } else if (gamepad1.y) {
            wrist.setPower(-1);
        } else if (gamepad1.x == false && gamepad1.y == false) {
            wrist.setPower(0);
        }
        if (gamepad1.a) {
            grabbie.setPower(1);
        } else if (gamepad1.b) {
            grabbie.setPower(-1);
        } else if (gamepad1.a == false && gamepad1.b == false) {
            grabbie.setPower(0);
        }
        if (gamepad1.dpad_left) {
            launcher.setPower(1);
        } else {
            launcher.setPower(0);
        }

        }
    }


