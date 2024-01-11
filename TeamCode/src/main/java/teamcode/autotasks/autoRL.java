//package teamcode.autotasks;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//import teamcode.vision.TrianglynotDuckyVisionRL;
//
//import teamcode.vision.TrianglynotDuckyVisionRL.triangleLocation;
//
//@Autonomous(name="IMURL")
//public class autoRL extends LinearOpMode {
//    /* Declare OpMode members. */
//    public DcMotor[] drivetrain = new DcMotor[4];
//
//    public DcMotor motorFL, motorFR, motorBR, motorBL, body;
//    CRServo wrist, rightHanger, leftHanger;
//    CRServo launcher;
//    private CRServo grabbie;
//    private BNO055IMU imu = null;
//    private double          robotHeading  = 0;
//    private double          headingOffset = 0;
//    private double          headingError  = 0;
//
//    private double  targetHeading = 0;
//    private double  driveSpeed    = 0;
//    private double  turnSpeed     = 0;
//    private double  leftSpeed     = 0;
//    private double  rightSpeed    = 0;
//    private int     frontleftTarget    = 0;
//    private int     frontrightTarget   = 0;
//    private int     backleftTarget    = 0;
//    private int     backrightTarget   = 0;
//    Integer cpr = 570; //counts per rotation
//    Integer gearRatio = 1;
//    Double diameter = 4.125;
//    Double cpi = (cpr * gearRatio)/(Math.PI * diameter); //counts per inch
//    Double meccyBias = 0.9;
//    Double bias = 0.8; //default 0.8
//    Double conversions = cpi * bias;
//    Boolean exit = false;
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    Integer COUNTS_PER_MOTOR_REV = 570; //counts per rotation
//    Integer DRIVE_GEAR_REDUCTION = 1;
//    Double WHEEL_DIAMETER_INCHES = 4.125;
//    Double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(Math.PI * WHEEL_DIAMETER_INCHES); //counts per inch
//    // These constants define the desired driving/control characteristics
//    // They can/should be tweaked to suit the specific robot drive train.
//    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
//    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
//    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
//    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//    // Define the Proportional control coefficient (or GAIN) for "heading control".
//    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
//    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
//    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
//    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
//    static final double P_DRIVE_COEFF = 0.15;
//    grabbie = hardwareMap.get(CRServo.class, "grabbie");
//    rightHanger = hardwareMap.get(CRServo.class, "rightHanger");
//    leftHanger = hardwareMap.get(CRServo.class, "leftHanger");
//    launcher = hardwareMap.get(CRServo.class, "launcher");
//
//        motorBL.setDirection(DcMotor.Direction.REVERSE);
//        motorFL.setDirection(DcMotor.Direction.REVERSE);
//        motorBR.setDirection(DcMotor.Direction.FORWARD);
//        motorFR.setDirection(DcMotor.Direction.FORWARD);
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//
//    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//    imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        body.setDirection(DcMotorSimple.Direction.FORWARD);
////        arm.setDirection(DcMotorSimple.Direction.FORWARD);
//        body.setPower(0);
////        arm.setPower(0);
//
//        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        body.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    visionSystem = new TrianglynotDuckyVisionRL(this);
//
//
//        while (opModeInInit()) {
//        telemetry.update();
//    }
//
//    // Set the encoders for closed loop speed control, and reset the heading.
//        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        body.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    resetHeading();
//
//
//    waitForStart();
//    triangleLocation detectedLocation = visionSystem.getCurrentDetection();
//
//
//        switch (detectedLocation) {
//        case LEFT:
//            closeGrabbie();
//            IMUDrive(.5,12,0);
//            IMUTurn(.5, 32);
//            IMUHold(0.5,32, 1);
//            IMUDrive(.5,14.5,32);
//            IMUDrive(.5,-23, 32);
//            IMUTurn(.5,-90);
//            IMUHold(0.5,-90, 1);
//            IMUDrive(.5,84.5,-90);
//            IMUHold(0.5,-90, 1);
//            Strafe(0.5,49);
//            grabbie.setPower(1);
//            moveArm(-0.5, 30);
//            sleep(100);
//            moveWrist(1,800);
//            sleep(500);
//            grabbie.setPower(-1);
//            sleep(550);
//            moveWrist(-1,500);
//            break;
//        case MIDDLE:
//            closeGrabbie();
//            IMUDrive(.4,35.5,0);
//            IMUHold(.5,0,1);
//            IMUDrive(.4,-30,0);
//            IMUTurn(0.5,-90);
//            IMUHold(0.5,-90, 1);
//            IMUDrive(0.5,89, -90);
//            IMUHold(0.5,-90, 1);
//            Strafe(0.5,41);
//            grabbie.setPower(1);
//            moveArm(-0.5, 30);
//            sleep(100);
//            moveWrist(1,800);
//            sleep(500);
//            grabbie.setPower(-1);
//            sleep(550);
//            moveWrist(-1,500);
//            break;
//        case RIGHT:
//            closeGrabbie();
//            IMUDrive(.4,16,0);
//            IMUHold(.4,0,1);
//            IMUTurn(.4,-32);
//            IMUHold(.4,-32,1);
//            IMUDrive(.4,16.5,-32);
//            IMUHold(.4,-32,1);
//            IMUDrive(.4,-29,-32);
//            IMUTurn(.4,-90);
//            IMUHold(.4,-90,1);
//            IMUDrive(.4,98.5,-90);
//            IMUHold(.4,-90,1);
//            Strafe(0.5,23);
//            grabbie.setPower(1);
//            moveArm(-0.5, 30);
//            sleep(100);
//            moveWrist(1,800);
//            sleep(500);
//            grabbie.setPower(-1);
//            sleep(550);
//            moveWrist(-1,500);
//            break;
//    }
//
//}
//
//        motorFL.setPower(speed);
//                motorBL.setPower(speed);
//                motorFR.setPower(speed);
//                motorBR.setPower(speed);
//
//                while ((motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())) {
//                double error = getError(angle);
//                double steer = getSteer(error, P_DRIVE_COEFF);
//
//                if (inches < 0) steer *= -1.0;
//
//        double frontLeftSpeed = speed - steer;
//        double backLeftSpeed = speed + steer;
//        double backRightSpeed = speed + steer;
//        double frontRightSpeed = speed - steer;
//
//        double max = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed)), Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed)));
//        if (max > 1.0) {
//        frontLeftSpeed /= max;
//        frontRightSpeed /= max;
//        backLeftSpeed /= max;
//        backRightSpeed /= max;
//        }
//
//        motorFL.setPower(frontLeftSpeed);
//        motorFR.setPower(frontRightSpeed);
//        motorBL.setPower(backLeftSpeed);
//        motorBR.setPower(backRightSpeed);
//
//        }
//
//        motorFR.setPower(0);
//        motorFL.setPower(0);
//        motorBR.setPower(0);
//        motorBL.setPower(0);
//        }
//private double getError(double targetAngle) {
//        double robotError = targetAngle - getCurrentHeading();
//        while (robotError > 180) robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//        }
//
//private double getCurrentHeading() {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
//        }
//        *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//        *                   If a relative angle is required, add/subtract from current heading.
//        * @param holdTime   Length of time (in seconds) to hold the specified heading.
//        */
//public void IMUHold(double maxTurnSpeed, double heading, double holdTime) {
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        // keep looping while we have time remaining.
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//        // Determine required steering to keep on heading
//        turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//        // Clip the speed to the maximum permitted value.
//        turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//        // Pivot in place by applying the turning correction
//        moveRobot(0, turnSpeed);
//        // Display drive status for the driver.
//        sendTelemetry(false);
//        }
//        // Stop all motion;
//        moveRobot(0, 0);
//        }
//// **********  LOW Level driving functions.  ********************
////    /**
////     * This method uses a Proportional Controller to determine how much steering correction is required.
////     *
////     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
////     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
////     * @return                      Turning power needed to get to required heading.
////     */
////    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
////        targetHeading = desiredHeading;  // Save for telemetry
////        // Get the robot heading by applying an offset to the IMU heading
////        robotHeading = getRawHeading() - headingOffset;
////        // Determine the heading current error
////        headingError = targetHeading - robotHeading;
////        // Normalize the error to be within +/- 180 degrees
////        while (headingError > 180)  headingError -= 360;
////        while (headingError <= -180) headingError += 360;
////        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
////        return Range.clip(headingError * proportionalGain, -1, 1);
////    }
////    /**
////     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
////     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
////     * @param drive forward motor speed
////     * @param turn  clockwise turning motor speed.
////     */
////    public void moveRobot(double drive, double turn) {
////        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
////        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
////        leftSpeed  = drive - turn;
////        rightSpeed = drive + turn;
////        // Scale speeds down if either one exceeds +/- 1.0;
////        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
////        if (max > 1.0)
////        {
////            leftSpeed /= max;
////            rightSpeed /= max;
////        }
////        motorFL.setPower(leftSpeed);
////        motorFR.setPower(rightSpeed);
////        motorBL.setPower(leftSpeed);
////        motorBR.setPower(rightSpeed);
////    }
////    /**
////     *  Display the various control parameters while driving
////     *
////     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
////     */
////    private void sendTelemetry(boolean straight) {
////        if (straight) {
////            telemetry.addData("Motion", "Drive Straight");
////            telemetry.addData("Target Pos L:R",  "%7d:%7d",      frontleftTarget,  frontrightTarget, backleftTarget, backrightTarget);
////            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      motorFL.getCurrentPosition(),
////                    motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
////        } else {
////            telemetry.addData("Motion", "Turning");
////        }
////        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
////        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
////        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
////        telemetry.update();
////    }
////    /**
////     * read the raw (un-offset Gyro heading) directly from the IMU
////     */
////    public double getRawHeading() {
////        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////        return angles.firstAngle;
////    }
////    /**
////     * Reset the "offset" heading back to zero
////     */
////    public void resetHeading() {
////        // Save a new heading offset equal to the current raw heading.
////        headingOffset = getRawHeading();
////        robotHeading = 0;
//        }
//public void Strafe (double power,double inches)
//        {
//        int move = (int)(Math.round(inches * conversions));
//        motorFL.setTargetPosition(motorFL.getCurrentPosition() - move);
//        motorFR.setTargetPosition(motorFR.getCurrentPosition() + move);
//        motorBL.setTargetPosition(motorBL.getCurrentPosition() + move);
//        motorBR.setTargetPosition(motorBR.getCurrentPosition() -move);
//        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFL.setPower(-power);
//        motorFR.setPower(-power);
//        motorBR.setPower(-power);
//        motorBL.setPower(-power);
//        while (motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy())
//        {
//        if(exit)
//        {
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//        motorFR.setPower(0);
//        motorFL.setPower(0);
//        }
//        }
//        }
//
//public void closeGrabbie()
//        {
//        grabbie.setPower(1);
//        sleep(500);
//        }
//public void openGrabbie ()
//        {
//        grabbie.setPower(-1);
//        sleep(500);
//        }
//public void moveWrist(int power, int time)
//        {
//        wrist.setPower(-power);
//        sleep(time);
//        wrist.setPower(0);
//        }
//public void moveArm (double power, double inches)
//        {
//        int move = (int)(Math.round(inches * conversions));
//        body.setTargetPosition(body.getCurrentPosition() +move);
//        body.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        body.setPower(power);
//        while (body.isBusy())
//        {
//        if(exit)
//        {
//        body.setPower(0);
//        }
//        }
//        }
//        }