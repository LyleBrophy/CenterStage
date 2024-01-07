package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.vision.TrianglynotDuckyVisionBL;

@TeleOp (name = "crying")
public class hangTest extends LinearOpMode {
    public DcMotor motorFL, motorFR, motorBR, motorBL;

    public DcMotor[] drivetrain = new DcMotor[4];
    DcMotor body;
    Dcmotor wrist;
    CRServo rightHanger, leftHanger;
    CRServo launcher;
    CRServo grabbie;

    public double speedMode = 0.7;


    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBR");
        body = hardwareMap.get(DcMotor.class, "body");
//        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        grabbie = hardwareMap.get(CRServo.class, "grabbie");
        rightHanger = hardwareMap.get(CRServo.class, "rightHanger");
        leftHanger = hardwareMap.get(CRServo.class, "leftHanger");
        launcher = hardwareMap.get(CRServo.class, "launcher");

        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        wrist.setPower(0);

        launcher.setPower(0);
        rightHanger.setPower(0);
        leftHanger.setPower(0);

        wrist.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setPower(0);

        body.setDirection(DcMotorSimple.Direction.FORWARD);
        body.setPower(0);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        body.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrianglynotDuckyVisionBL detector = new TrianglynotDuckyVisionBL(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speedMode = 1;
            } else if (gamepad1.right_bumper) {
                speedMode = 0.2;
            } else if (gamepad1.a) {
                speedMode = 0.7;
            } else {
                speedMode = 0.7;
            }
            double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
            double strafe = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
            double turn = -speedMode * Math.pow(gamepad1.right_stick_x, 3);
            double frontLeftPower = forward + strafe + turn;
            double backLeftPower = forward - strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backRightPower = forward + strafe - turn;
            double[] powers = {frontLeftPower, backLeftPower, frontRightPower, backRightPower};
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
                frontLeftPower /= greatest;
                backLeftPower /= greatest;
                frontRightPower /= greatest;
                backRightPower /= greatest;
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
                frontLeftPower = 0;
                backLeftPower = 0;
                frontRightPower = 0;
                backRightPower = 0;
            }
            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);
//            if (gamepad1.left_stick_y < -0.1) {
//                motorFL.setPower(1);
//                motorBR.setPower(1);
//                motorFR.setPower(1);
//                motorBL.setPower(1);
//
//            } else if (gamepad1.left_stick_y > 0.1) {
//                motorFL.setPower(-1);
//                motorBR.setPower(-1);
//                motorFR.setPower(-1);
//                motorBL.setPower(-1);
//            } else if (gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_y > -0.1) {
//                motorFL.setPower(0);
//                motorBR.setPower(0);
//                motorFR.setPower(0);
//                motorBL.setPower(0);
//            }
//            if (gamepad1.left_stick_x > 0.1) {
//                motorFL.setPower(1);
//                motorBR.setPower(1);
//                motorFR.setPower(-1);
//                motorBL.setPower(-1);
//            } else if (gamepad1.left_stick_x < -0.1) {
//                motorFL.setPower(-1);
//                motorBR.setPower(-1);
//                motorFR.setPower(1);
//                motorBL.setPower(1);
//            } else if (gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_x < 0.1) {
//                motorFL.setPower(0);
//                motorBR.setPower(0);
//                motorFR.setPower(0);
//                motorBL.setPower(0);
//            }
//            if (gamepad1.right_stick_x > 0.1) {
//                motorFL.setPower(1);
//                motorBL.setPower(1);
//                motorBR.setPower(-1);
//                motorFR.setPower(-1);
//            } else if (gamepad1.right_stick_x < -0.1) {
//                motorFL.setPower(-1);
//                motorBL.setPower(-1);
//                motorBR.setPower(1);
//                motorFR.setPower(1);
//            } else if (gamepad1.right_stick_x > -0.1 && gamepad1.right_stick_x < 0.1) {
//                motorFL.setPower(0);
//                motorBR.setPower(0);
//                motorFR.setPower(0);
//                motorBL.setPower(0);
//            }
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

            if (gamepad2.left_stick_y < -0.1) {
                body.setPower(-1);
            } else if (gamepad2.left_stick_y > 0.1) {
                body.setPower(0.4);
            } else if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                body.setPower(0);
            }
//            if (gamepad2.right_stick_y < -0.1) {
//                arm.setPower(1);
//            } else if (gamepad2.right_stick_y > 0.1) {
//                arm.setPower(-1);
//            } else if (gamepad2.right_stick_y < 0.1 && gamepad2.right_stick_y < -0.1) {
//                arm.setPower(0);
//            } else {
//                arm.setPower(0);
//            }
            if (gamepad2.dpad_up) {
                wrist.setPower(0.5);
            } else if (gamepad2.dpad_down) {
                wrist.setPower(-0.2);
            } else if (gamepad2.dpad_up == false && gamepad2.dpad_down == false) {
                wrist.setPower(0);
            }
            if (gamepad2.x) {
                grabbie.setPower(-1);
            } else if (gamepad2.y) {
                grabbie.setPower(1);
            } else if (gamepad2.x == false && gamepad2.y == false) {
                grabbie.setPower(0);
            }
            if (gamepad2.left_bumper == true && gamepad2.right_bumper == true) {
                launcher.setPower(1);
            } else {
                launcher.setPower(0);
            }
            if (gamepad1.dpad_left) {
                leftHanger.setPower(-1);
                rightHanger.setPower(1);
                sleep(600);
                while (opModeIsActive()) {
                    leftHanger.setPower(1);
                    rightHanger.setPower(-1);
                    sleep(300);
                    leftHanger.setPower(-1);
                    rightHanger.setPower(1);
                    sleep(1000);
                    telemetry.addLine("This is gonna work, trust");
                    telemetry.update();
                }
                if (gamepad1.x && gamepad1.y) {
                    launcher.setPower(1);
                }else {
                    launcher.setPower(0);
                }
//                if (gamepad1.left_bumper) {
//                    speedMode = 1;
//                } else if (gamepad1.right_bumper) {
//                    speedMode = 0.2;
//                } else if (gamepad1.a) {
//                    speedMode = 0.7;
//                }

            }
        }
    }
}

