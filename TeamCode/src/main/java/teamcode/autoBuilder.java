package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="autobuilder")
@Disabled
public class autoBuilder extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor Lift;
    CRServo Grabbie;

    ModernRoboticsI2cGyro gyro;


    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    Integer cpr = 13; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    double amountError = 2;
    Double conversions = cpi * bias;


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    double DRIVE_SPEED = 0.45;
    double TURN_SPEED = 0.4;

    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void runOpMode() throws InterruptedException {

        initGyro();

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Grabbie = hardwareMap.get(CRServo.class, "Grabbie");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");


        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        Lift.setDirection(DcMotor.Direction.FORWARD);


        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        Lift.setPower(0);
        Grabbie.setPower(0);

        telemetry.addLine("Start Gyro");
        telemetry.update();
        gyro.calibrate();
        while (gyro.isCalibrating()) ;
        telemetry.addLine("Gyro Calibrated");
//        RedLeftVision detector = new RedLeftVision(this);
        while (!isStarted()) {
            telemetry.addData("Angle: ", gyro.getIntegratedZValue());
//            telemetry.addData("Red", detector.one);
//            telemetry.addData("Green", detector.two);
//            telemetry.addData("Blue", detector.three);
            telemetry.update();
        }

        waitForStart();
        if (gamepad1.left_stick_x < 0) {
            motorBL.setPower(-1);
            motorFR.setPower(-1);
            motorBR.setPower(1);
            motorFL.setPower(1);
        } else if (gamepad1.left_stick_x > 0) {

        }
    }

    public void drivebackrightandfrontleft(double inches, double speed) {

        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        motorBL.setTargetPosition(motorBL.getCurrentPosition() - move);
        //frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        //backright.setTargetPosition(backright.getCurrentPosition() + move);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - move);
        //
        //frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        //frontleft.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        //backright.setPower(speed);
        //
        while (motorFR.isBusy() && motorBL.isBusy()) {
        }
        motorFR.setPower(0);
        //frontleft.setPower(0);
        //backright.setPower(0);
        motorBL.setPower(0);
        return;

    }

    public void MoveLift(double inches, double speed) {
        int move = (int) (Math.round(inches * conversions));

        Lift.setTargetPosition(Lift.getCurrentPosition() + move);

        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift.setPower(speed);

        while (Lift.isBusy()) {
            if (exit) {
                Lift.setPower(0);
            }
        }
    }


    public void turnWithGyro(double degrees, double speedDirection) {
        //<editor-fold desc="Initialize">
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0) {//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            } else {
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        } else {
            //<editor-fold desc="turn left">
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            } else {
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
//        TurnWithEncoder(double speed, double Direction);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        } else {
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        TurnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            motorFL.setPower(0);
//            motorFR.setPower(0);
//            motorBL.setPower(0);
//            motorBR.setPower(0);
//        }
        //</editor-fold>
        //
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void StrafeToPosition(double inches, double angle, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //\
        int newFrontLeftTarget = motorFL.getCurrentPosition() + move;
        int newFrontRightTarget = motorFR.getCurrentPosition() - move;
        int newBackLeftTarget = motorBL.getCurrentPosition() - move;
        int newBackRightTarget = motorBR.getCurrentPosition() + move;
        motorBL.setTargetPosition(newBackLeftTarget);
        motorFL.setTargetPosition(newFrontLeftTarget);
        motorBR.setTargetPosition(newBackRightTarget);
        motorFR.setTargetPosition(newFrontRightTarget);


        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;
        //
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);
        //
        while ((motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) && !goodEnough) {
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                steer *= -1.0;

            frontLeftSpeed = speed - steer;
            backLeftSpeed = speed + steer;
            backRightSpeed = speed + steer;
            frontRightSpeed = speed - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
            HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
            max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
            if (max > 1.0) {
                frontLeftSpeed /= max; //frontLeftSpeed = frontLeftSpeed / max += -= *= %=
                frontRightSpeed /= max;
                backLeftSpeed /= max;
                backRightSpeed /= max;
            }

            motorFL.setPower(frontLeftSpeed);
            motorFR.setPower(frontRightSpeed);
            motorBL.setPower(backLeftSpeed);
            motorBR.setPower(backRightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
            telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
            telemetry.addData("Actual", "%7d:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition(), motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
            telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
            telemetry.update();

            ErrorAmount = ((Math.abs(((newBackLeftTarget) - (motorBL.getCurrentPosition())))
                    + (Math.abs(((newFrontLeftTarget) - (motorFL.getCurrentPosition()))))
                    + (Math.abs((newBackRightTarget) - (motorBR.getCurrentPosition())))
                    + (Math.abs(((newFrontRightTarget) - (motorFR.getCurrentPosition()))))) / cpi);
            if (ErrorAmount < amountError) {
                goodEnough = true;
            }


        }
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void TurnWithEncoder(double power, double inches) {
        int move = (int) (Math.round(inches * conversions));

        motorFL.setTargetPosition(motorFL.getCurrentPosition() - move);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() + move);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() - move);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() + move);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(-power);
        motorFR.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);

        while (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) {
            if (exit) {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFR.setPower(0);
                motorFL.setPower(0);
            }
        }
    }

    public void GyroDrive(double speed,
                          double frontLeftInches, double frontRightInches, double backLeftInches,
                          double backRightInches,
                          double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = motorFL.getCurrentPosition() + (int) (frontLeftInches * cpi);
            newFrontRightTarget = motorFR.getCurrentPosition() + (int) (frontRightInches * cpi);
            newBackLeftTarget = motorBL.getCurrentPosition() + (int) (backLeftInches * cpi);
            newBackRightTarget = motorBR.getCurrentPosition() + (int) (backRightInches * cpi);


            // Set Target and Turn On RUN_TO_POSITION
            motorFL.setTargetPosition(newFrontLeftTarget);
            motorFR.setTargetPosition(newFrontRightTarget);
            motorBL.setTargetPosition(newBackLeftTarget);
            motorBR.setTargetPosition(newBackRightTarget);

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFL.setPower(Math.abs(speed));
            motorFR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    ((motorFL.isBusy() && motorFR.isBusy()) && (motorBL.isBusy() && motorBR.isBusy())) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max; //frontLeftSpeed = frontLeftSpeed / max += -= *= %=
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                motorFL.setPower(frontLeftSpeed);
                motorFR.setPower(frontRightSpeed);
                motorBL.setPower(backLeftSpeed);
                motorBR.setPower(backRightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Actual", "%7d:%7d", motorBL.getCurrentPosition(), motorBR.getCurrentPosition(), motorFL.getCurrentPosition(), motorFR.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (motorBL.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (motorFL.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (motorBR.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (motorFR.getCurrentPosition()))))) / cpi);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void GyroTurn(double speed, double angle) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */


    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();


        }

        // Stop all motion;
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double LeftSpeed;
        double RightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            LeftSpeed = 0.0;
            RightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            RightSpeed = speed * steer;
            LeftSpeed = -RightSpeed;
        }

        // Send desired speeds to motors.
        motorFL.setPower(LeftSpeed);
        motorBL.setPower(LeftSpeed);
        motorBR.setPower(RightSpeed);
        motorFR.setPower(RightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", LeftSpeed, RightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return -robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -DRIVE_SPEED, 1);
    }

    public void OpenGrabbie() {
        Grabbie.setPower(-1);
        sleep(150);
        Grabbie.setPower(0);
    }

    public void CloseGrabbie() {
        Grabbie.setPower(1);
        sleep(250);
    }
}