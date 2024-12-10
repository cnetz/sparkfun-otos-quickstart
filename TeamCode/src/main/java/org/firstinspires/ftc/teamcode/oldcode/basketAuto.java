package org.firstinspires.ftc.teamcode.oldcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class basketAuto extends OpMode {
    double driveTolerance = 20;
    double currentInches = 0;
    double frontLeftDistance = 0;
    double backLeftDistance = 0;
    double backRightDistance = 0;
    double frontRightDistance = 0;
    double baseSpeed = 0.1;
    double maxSpeed = 0.7;

    //PID Controller

    private PIDController armController;
    private PIDController slideController;
    double armP = 0.004, armI = 0, armD = 0.0002;
    double armF = 0.03;
    int armTarget = 0;
    int armThreshold = 10;
    double armTicksInDegree = 285 / 180; //1425 / 5 (gear ratio) = 285
    double slideP = 0.006, slideI = 0, slideD = 0.0001;
    double slideF = 0.04;
    int slideTarget = 0;
    int slideThreshold = 50;
    double slideTicksInDegree = 358.466 / 180;

    //Drive conversion

    double cpr = 537.7;
    double gearRatio = 1;
    double diameter = 4.1; //4.09449 in inches - mecanum
    double slideDiameter = 1.5;
    double cpi = (cpr * gearRatio) / (Math.PI * diameter);
    double cpiSlide = (cpr * gearRatio) / (Math.PI * slideDiameter);
    double bias = 1.0;
    double conversion = cpi * bias;
    double robotWidth = 12.9;
    double turnCF = Math.PI * robotWidth;

    //Motors, servos, etc

    private DcMotorEx jointMotor, slideMotor;
    private DcMotor frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor;
    private Servo wristServo, clawServo, basketServo;
    private final ElapsedTime newTimer = new ElapsedTime();
    private final ElapsedTime delayTimer = new ElapsedTime();
    private boolean isDelaying = false;
    public ElapsedTime delay = null;

    IMU imu;
    private boolean fiveSeconds = false;
    private boolean exit = false;

    //States

    private enum OrderState {
        FIRST, SECOND, THIRD, FOURTH, FIFTH, SIX, SEVEN, EIGHT, NINE, TEN
    }

    private OrderState currentOrderState = OrderState.FIRST;

    private enum SlideState {
        IDLE, MOVING, COMPLETED
    }

    private SlideState currentSlideState = SlideState.IDLE;

    private enum DriveState {
        IDLE, MOVING, COMPLETED
    }

    private DriveState currentDriveState = DriveState.IDLE;

    private enum armState {
        IDLE, COMPLETED, MOVING//same as completed
    }

    private armState currentArmState = armState.IDLE;

    private enum StrafeState {
        IDLE, MOVING, COMPLETED
    }

    private StrafeState currentStrafeState = StrafeState.IDLE;

    //Current Step

    private int currentStep = 0;

    private boolean test = false;

    @Override
    public void init() {
        armController = new PIDController(armP, armI, armD); //Declares PID Controller for arm/joint
        slideController = new PIDController(slideP, slideI, slideD); //Declares PID Controller for slide
        ElapsedTime delay = null;

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor"); // EXP 2
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor"); // EXP 3
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor"); // EXP 0
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor"); // EXP 1
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor"); // CON - 3
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor"); // CON - 0
        wristServo = hardwareMap.get(Servo.class, "wristServo"); // CON - 0
        clawServo = hardwareMap.get(Servo.class, "clawServo"); // EXP - 5
        basketServo = hardwareMap.get(Servo.class, "basketServo"); // CON - 4
        imu = hardwareMap.get(IMU.class, "imu");

        //Sets motor power behavior to brake

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets motor directions

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reset and declare motor modes

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //IMU orientation

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void init_loop() {
        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        } else {
            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        }
        updateTelemetry();
    }

    @Override
    public void start() {
        clawServo.setPosition(0.85);
        basketServo.setPosition(0.3);
        newTimer.reset();
    }

    @Override
    public void loop() {
        updateTelemetry();


        if (!test) {
            switch (currentStep) {
                case 0: // drives 8 inches and sets basket servo
                    if ((currentDriveState == DriveState.IDLE)) {
                        moveToPos(8, 0.6);
                        basketServo.setPosition(0.3);
                    }
                    if ((currentDriveState == DriveState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        currentStep++;
                    }
                    break;
                case 1://strafe -30 inches and set slide to 2800
                    if ((currentStrafeState == StrafeState.IDLE) && (currentSlideState == SlideState.IDLE)) {
                        setTargetSlide(2800);
                        strafeToPos(-15, 0.55);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED) && (currentSlideState == SlideState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentSlideState = SlideState.IDLE;
                        currentStep++;
                    }
                    break;
                case 2: //Drive -15 inches to aline to place specimen
                    if ((currentArmState == armState.IDLE)) {
                        setTargetArm(2300);
                        wristServo.setPosition(0.4);
                    }
                    if ((currentArmState == armState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 3: //Drives 1.5 to to place speciman
                    if ((currentStrafeState == StrafeState.IDLE) && (currentSlideState == SlideState.IDLE)) {
                        strafeToPos(-9, 0.5);
                        setTargetSlide(3300);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED) && (currentSlideState == SlideState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentSlideState = SlideState.IDLE;
                        currentStep++;
                    }
                    break;
                case 4: //Move arm to place specimen
                    if ((currentDriveState == DriveState.IDLE)) {
                        moveToPos(-5, 0.5);
                    }
                    if ((currentDriveState == DriveState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        basketServo.setPosition(0.7);
                        currentStep++;
                    }
                    break;
                case 5:
                    if (customDelay(0.5)) {
                        currentStep++;
                    }
                    break;
                case 6: // BIG Strafe and lift arm (arm all the way up)
                    if ((currentStrafeState == StrafeState.IDLE)) {
                        strafeToPos(7.5, 0.5);
                        basketServo.setPosition(0.3);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentStep++;
                    }
                    break;
                case 7: //Lower arm + claw to pickup 2nd specimen
                    if ((currentArmState == armState.IDLE)) {
                        setTargetArm(4925);
                        wristServo.setPosition(0.40);
                        clawServo.setPosition(0.5);
                    }
                    if ((currentArmState == armState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 8: //Drive forward 8 inches then close claw
                    if ((currentDriveState == DriveState.IDLE) && (currentSlideState == SlideState.IDLE)) {
                        moveToPos(18, 0.3);
                        setTargetSlide(800);
                    }
                    if ((currentDriveState == DriveState.COMPLETED) && (currentSlideState == SlideState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        currentSlideState = SlideState.IDLE;
                        clawServo.setPosition(0.72);
                        currentStep++;
                    }

                    break;
                case 9: //Drive 4 inches and lift arm
                    if ((currentDriveState == DriveState.IDLE)) {
                        moveToPos(1, 0.1);
                    }
                    if ((currentDriveState == DriveState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        currentStep++;
                    }

                    break;
                case 10: //Drive backwards -4 inches
                    if ((currentArmState == armState.IDLE) && (currentDriveState == DriveState.IDLE)) {
                        setTargetArm(2300);
                        moveToPos(-19, 0.5);
                        wristServo.setPosition(0.1);
                    }
                    if ((currentArmState == armState.COMPLETED) && (currentDriveState == DriveState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentDriveState = DriveState.IDLE;
                        clawServo.setPosition(0.5);
                        currentStep++;
                    }
                    break;
                case 11:
                    if (customDelay(0.5)) {
                        currentStep++;
                    }
                    break;
                case 12:
                    if (customDelay(0.5)) {
                        currentStep++;
                    }
                    clawServo.setPosition(0.5);


                    break;
                case 13: //Strafe 44 inches to place next specimen
                    if ((currentArmState == armState.IDLE)) {
                        setTargetArm(2400);
                        wristServo.setPosition(0.4);
                    }
                    if ((currentArmState == armState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 14: //Drive -5 and move arm to place
                    if ((currentSlideState == SlideState.IDLE)) {
                        setTargetSlide(3300);
                    }
                    if ((currentSlideState == SlideState.COMPLETED)) {
                        currentSlideState = SlideState.IDLE;
                        currentStep++;
                    }
                    break;
                case 15:// set arm and claw to place
                    if ((currentStrafeState == StrafeState.IDLE)) {
                        strafeToPos(-6.5, 0.5);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentStep++;
                    }
                    break;
                case 16:
                    if (customDelay(0.2)) {
                        currentStep = 18;
                    }

                    break;
                case 18:
                    basketServo.setPosition(0.7);
                    if (customDelay(0.2)) {
                        currentStep++;
                    }
                    break;
                case 19:
                    if (currentStrafeState == StrafeState.IDLE) {
                        strafeToPos(16, 0.5);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentStep++;
                    }
                    break;
                case 20:
                    if ((currentSlideState == SlideState.IDLE) && (currentArmState == armState.IDLE)) {
                        setTargetSlide(800);
                        setTargetArm(4925);
                        wristServo.setPosition(0.4);
                        basketServo.setPosition(0.3);
                    }
                    if ((currentSlideState == SlideState.COMPLETED) && (currentArmState == armState.COMPLETED)) {
                        currentSlideState = SlideState.IDLE;
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 21:
                    if ((currentDriveState == DriveState.IDLE)) {
                        moveToPos(19, 0.4);
                    }
                    if ((currentDriveState == DriveState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        clawServo.setPosition(0.72);
                        currentStep++;
                    }
                    break;
                case 22:
                    if ((currentDriveState == DriveState.IDLE)) {
                        moveToPos(1, 0.1);
                    }
                    if ((currentDriveState == DriveState.COMPLETED)) {
                        currentDriveState = DriveState.IDLE;
                        currentStep++;
                    }
                    break;
                case 23: //Drive backwards -4 inches
                    if ((currentArmState == armState.IDLE) && (currentDriveState == DriveState.IDLE)) {
                        setTargetArm(2300);
                        moveToPos(-19, 0.5);
                        wristServo.setPosition(0.1);
                    }
                    if ((currentArmState == armState.COMPLETED) && (currentDriveState == DriveState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentDriveState = DriveState.IDLE;
                        clawServo.setPosition(0.5);
                        currentStep++;
                    }
                    break;
                case 24:
                    if (customDelay(0.5)) {
                        currentStep++;
                    }
                    break;
                case 25:
                    if (customDelay(0.5)) {
                        currentStep++;
                    }
                    clawServo.setPosition(0.5);


                    break;
                case 26: //Strafe 44 inches to place next specimen
                    if ((currentArmState == armState.IDLE)) {
                        setTargetArm(2400);
                        wristServo.setPosition(0.4);
                    }
                    if ((currentArmState == armState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 27: //Drive -5 and move arm to place
                    if ((currentSlideState == SlideState.IDLE)) {
                        setTargetSlide(3300);
                    }
                    if ((currentSlideState == SlideState.COMPLETED)) {
                        currentSlideState = SlideState.IDLE;
                        currentStep++;
                    }
                    break;
                case 28:// set arm and claw to place
                    if ((currentStrafeState == StrafeState.IDLE)) {
                        strafeToPos(-16.5, 0.5);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentStep++;
                    }
                    break;
                case 29:
                    if (customDelay(0.2)) {
                        currentStep++;
                    }

                    break;
                case 30:
                    basketServo.setPosition(0.7);
                    if (customDelay(0.2)) {
                        currentStep++;
                    }
                    break;
                case 31: //Drive -5 and move arm to place
                    if ((currentStrafeState == StrafeState.IDLE)) {
                        strafeToPos(5, 0.5);
                    }
                    if ((currentStrafeState == StrafeState.COMPLETED)) {
                        currentStrafeState = StrafeState.IDLE;
                        currentStep++;
                    }
            }
        } else { // TESTING
            switch (currentStep) {
                case 0:
                    if ((currentArmState == armState.IDLE)) {
                        setTargetArm(200);
                    }
                    if ((currentArmState == armState.COMPLETED)) {
                        currentArmState = armState.IDLE;
                        currentStep++;
                    }
                    break;
                case 1:
                    break;
            }
        }
        strafeFSM();
        driveFSM();
        armFSM();
        slideFSM();
    }

    @Override
    public void stop() {
        exit = true;
    }

    public void updateTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("time", newTimer.seconds());
        telemetry.addData("inches", currentInches);
        telemetry.addData("frontLeftDistance", frontLeftDistance);
        telemetry.addData("frontLeftPower", frontLeftMotor.getPower());
        telemetry.addData("frontLeftPos", frontLeftMotor.getCurrentPosition());
/*        telemetry.addData("frontRight", frontRightMotor.getCurrentPosition());
        telemetry.addData("backLeft", backLeftMotor.getCurrentPosition());
        telemetry.addData("backRight", backRightMotor.getCurrentPosition());
        telemetry.addData("DriveState", currentDriveState);
        telemetry.addData("SlideState", currentSlideState);
        telemetry.addData("ArmState", currentArmState);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("jointPos", jointMotor.getCurrentPosition());
        telemetry.addData("slideTarget", slideTarget);
        telemetry.addData("slidePos", slideMotor.getCurrentPosition());
        telemetry.addData("jointPower", jointMotor.getPower());*/
        telemetry.update();
    }

    public void moveToPos(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));
/*        currentInches = Math.abs(inches);
        if (currentInches > 5) {
            frontLeftDistance = (frontLeftMotor.getCurrentPosition() + move);
            backLeftDistance = (backLeftMotor.getCurrentPosition() + move);
            backRightDistance = (backRightMotor.getCurrentPosition() + move);
            frontRightDistance = (frontRightMotor.getCurrentPosition() + move);
        }*/

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        currentDriveState = DriveState.MOVING;

    }

    public void strafeToPos(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));

/*        currentInches = Math.abs(inches);
        if (currentInches > 5) {
            frontLeftDistance = (frontLeftMotor.getCurrentPosition() + move);
            backLeftDistance = (backLeftMotor.getCurrentPosition() - move);
            backRightDistance = (backRightMotor.getCurrentPosition() + move);
            frontRightDistance = (frontRightMotor.getCurrentPosition() - move);
        }*/

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + move);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - move);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - move);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + move);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        currentStrafeState = StrafeState.MOVING;

    }

    public void moveSlideNormal(double inches, double speed) {
        int move = (int) (Math.round(inches * cpiSlide));
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + move);

        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor.setPower(speed);
        currentSlideState = SlideState.MOVING;

    }

    public void moveJoint(double inches, double speed) {
        int move = (int) (Math.round(inches * cpiSlide * 3));
        jointMotor.setTargetPosition(move);

        jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jointMotor.setPower(speed);
    }

    public void setTargetArm(int target) {
        armTarget = target;
        currentArmState = armState.MOVING;
    }

    public void moveArm(int target) {
        armController.setPID(armP, armI, armD);
        int jointPos = jointMotor.getCurrentPosition();
        double pid = armController.calculate(jointPos, target);
        double ff = Math.cos(Math.toRadians(target / armTicksInDegree)) * armF;
        double power = pid + ff;
        jointMotor.setPower(power);
    }

    public void armFSM() {
        switch (currentArmState) {
            case IDLE:
                int currentPos1 = jointMotor.getCurrentPosition();
                moveArm(currentPos1);
                break;
            case COMPLETED:
                moveArm(armTarget);
                break;
            case MOVING:
                moveArm(armTarget);
                double currentPos2 = jointMotor.getCurrentPosition();

                if (Math.abs((armTarget - currentPos2)) < armThreshold) {
                    currentArmState = armState.COMPLETED;
                }

        }
    }

    public void setTargetSlide(int target) {
        slideTarget = target;
        currentSlideState = SlideState.MOVING;
    }

    public void moveSlide(int target) {
        slideController.setPID(slideP, slideI, slideD);
        int slidePos = slideMotor.getCurrentPosition();
        double pid = slideController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / slideTicksInDegree)) * slideF;
        double power = pid + ff;
        slideMotor.setPower(power);
    }

    public void slideFSM() {
        switch (currentSlideState) {
            case IDLE:
                int currentPos1 = slideMotor.getCurrentPosition();
                moveSlide(currentPos1);
                break;
            case COMPLETED:
                moveSlide(slideTarget);
                break;
            case MOVING:
                moveSlide(slideTarget);
                double currentPos2 = slideMotor.getCurrentPosition();

                if (Math.abs((slideTarget - currentPos2)) < slideThreshold) {
                    currentSlideState = SlideState.COMPLETED;
                }
        }
    }

    public void driveFSM() {
        switch (currentDriveState) {
            case IDLE:
                break;

            case MOVING:
                if ((Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) < driveTolerance)) {

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    currentDriveState = DriveState.COMPLETED;
                    break;
                }
                break;

            case COMPLETED:
                // Now transition back to IDLE for future movement
                break;
        }
    }

    public void strafeFSM() {
        switch (currentStrafeState) {
            case IDLE:
                break;

            case MOVING:
                if ((Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) < driveTolerance
                        && Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) < driveTolerance)) {

                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);

                    currentStrafeState = StrafeState.COMPLETED;
                    break;
                }
                break;

            case COMPLETED:
                // Now transition back to IDLE for future movement
                break;
        }
    }
    //private double calculateSinusoidalSpeed(int currentDistance, int totalDistance)
    //fractionTraveled = (double) currentDistance / totalDistance;
    //speed = baseSpeed + (maxSpeed - baseSpeed) * Math.sin(Math.PI * fractionTraveled);
    //return speed;

    //private double applyAccelerationControl(double currentSpeed, double targetSpeed, double maxAcceleration)
    // speedDifference = targetSpeed - currentSpeed
    // if(speedDifference > maxAcceleration) {
    // currentSpeed += maxAcceleration
    // } else if (speedDifference < -maxAcceleration) {
    // currentSpeed -= maxAcceleration
    // } else {
    // currentSpeed = targetSpeed;
    // }
    // return currentSpeed;

    private double calculateSpeed(int currentDistance, int totalDistance) {
        int distanceRemaining = Math.abs(totalDistance - currentDistance);
        int totalTravelDistance = Math.abs(totalDistance);

        double traveled = 1.0 - (double) (distanceRemaining / totalTravelDistance);
        traveled = Math.max(1, Math.max(0, traveled));

        return (baseSpeed + (maxSpeed - baseSpeed) * Math.sin(Math.PI * traveled));


    }

    private boolean customDelay(double seconds) {
        if (!isDelaying) {
            delayTimer.reset();//startTimer
            isDelaying = true;
        }
        if (delayTimer.seconds() >= seconds) {
            isDelaying = false;
            return true;
        }
        return false;//still delaying
    }
}