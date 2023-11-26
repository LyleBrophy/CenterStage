package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BasmalahBadTeleOp")
public class BasmalahBadTeleOp extends LinearOpMode {
    public DcMotor motorFL, motorFR, motorBR, motorBL;

    public DcMotor[] drivetrain = new DcMotor[4];
    DcMotor body;
    CRServo wrist, arm, rightHanger, leftHanger, grabbie;
    Servo launcher;

    public double speedMode = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBR");
        body = hardwareMap.get(DcMotor.class, "body");
        arm = hardwareMap.get(CRServo.class, "arm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        grabbie = hardwareMap.get(CRServo.class, "grabbie");
        rightHanger = hardwareMap.get(CRServo.class, "rightHanger");
        leftHanger = hardwareMap.get(CRServo.class, "leftHanger");
        launcher = hardwareMap.get(Servo.class, "launcher");

        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
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
            rightHanger.setPower(-1);
            leftHanger.setPower(1);
            telemetry.addLine("This is gonna work, trust");
            telemetry.update();
        } else if (gamepad1.right_trigger > 0.1) {
            rightHanger.setPower(1);
            leftHanger.setPower(-1);
            telemetry.addLine("This is gonna work, trust");
            telemetry.update();
        } else if (gamepad1.dpad_right) {
            leftHanger.setPower(0);
            rightHanger.setPower(0);
            telemetry.addLine("We hate Bessmellah");
            telemetry.update();
        }

        if (gamepad1.left_bumper) {
            body.setPower(1);
        } else if (gamepad1.right_bumper) {
            body.setPower(-1);
        } else if (gamepad1.left_bumper == false && gamepad1.right_bumper == false) {
            body.setPower(0);
        }
        if (gamepad1.dpad_up) {
            arm.setPower(1);
        } else if (gamepad1.dpad_down) {
            arm.setPower(-1);
        } else if (gamepad1.dpad_up == false && gamepad1.dpad_down == false) {
            arm.setPower(0);
        }
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
//        if (gamepad1.dpad_left) {
//            launcher.setPosition(1);
//        } else {
//            launcher.setPosition(0);
//        }
        if (gamepad1.dpad_left) {
            leftHanger.setPower(-1);
            rightHanger.setPower(1);
            sleep(2500);
            while (opModeIsActive()) {
                leftHanger.setPower(1);
                rightHanger.setPower(-1);
                sleep(300);
                leftHanger.setPower(-1);
                rightHanger.setPower(1);
                sleep(300);
                telemetry.addLine("This is gonna work, trust");
                telemetry.update();
            }
        }
        }
    }

