package org.firstinspires.ftc.teamcode.oldcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oldcode.ButtonHandler;

@TeleOp
public class mecanumDrive extends LinearOpMode {
    private RevBlinkinLedDriver lights;

    // Conversion
    double cpr = 537.7;
    double gearRatio = 1;
    double slideDiameter = 1.5;
    double cpiSlide = (cpr * gearRatio)/(Math.PI * slideDiameter);
    double bias = 1.1;

    //PID Controller

    private PIDController controller;
    public static double p = 0.004,i = 0, d = 0.0002;
    public static double f = 0.03;
    private final double ticksInDegree = (double) 285 / 180; //1425 / 5 (gear ratio) = 285
    int armTarget = 0;

    //HardwareMap

    private DcMotor frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor, slideMotor, jointMotor, hangMotor;
    private Servo wristServo, clawServo, basketServo, hangServo;

    //States

    private ElapsedTime newTimer = new ElapsedTime();
    private enum armState{
        IDLE, HOLDING,MOVING//same as completed
    }
    private armState currentArmState = armState.IDLE;
    private enum SlideState {
        COMPLETED,
        IDLE,
        MANUAL,
        MOVING
    }
    private SlideState currentSlideState = SlideState.IDLE;

    @Override
    public void runOpMode()  {

        //PID Controller

        controller = new PIDController(p,i,d);

        //HardwareMap

        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        jointMotor = hardwareMap.get(DcMotor.class, "jointMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo = hardwareMap.get(Servo.class,"wristServo");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        basketServo = hardwareMap.get(Servo.class,"basketServo");
        hangServo = hardwareMap.get(Servo.class,"hangServo");

        //Motor Power Behavior

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Motor Direction & Mode

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Lights

        lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");

        //Defaults

        double changeSpeed = 0.65;
        boolean changeSpeedPos = false;
        int armTempPos = 0;

        waitForStart();

        armTarget = jointMotor.getCurrentPosition();
        clawServo.setPosition(0.6);


        newTimer.reset();

        //Button Handler

        ButtonHandler buttonHandler = new ButtonHandler();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            //Updates

            lightsUpdate();
            armFSM();
            slideFSM();
            updateTelemetry();

            //Driving

            double y = gamepad1.left_stick_y; // set y to gamepad 1 left stick y
            double x = -gamepad1.left_stick_x * bias; // set x to gamepad 1 left stick x
            double rx = -gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // denominator for scaling, which cant go above 1
            double backRightPower = ((y + x - rx) / denominator); //Set back right power to (gamepad1.left_stick_y) + (-gamepad1.left_stick_x) - (-gamepad1.right_stick_x) / denominator
            double frontLeftPower = ((y + x + rx) / denominator); //Set front left power to (gamepad1.left_stick_y) + (-gamepad1.left_stick_x) + (-gamepad1.right_stick_x) / denominator
            double frontRightPower = ((y - x - rx) / denominator); //Set front right power to (gamepad1.left_stick_y) - (-gamepad1.left_stick_x) - (-gamepad1.right_stick_x) / denominator
            double backLeftPower = ((y - x + rx) / denominator); //Set back left power to (gamepad1.left_stick_y) - (-gamepad1.left_stick_x) + (-gamepad1.right_stick_x) / denominator

            //Set power for motors

            frontRightMotor.setPower(frontRightPower * changeSpeed);
            backRightMotor.setPower(backRightPower * changeSpeed);
            frontLeftMotor.setPower(frontLeftPower * changeSpeed);
            backLeftMotor.setPower(backLeftPower * changeSpeed);

            //Joint/Arm control

            if (gamepad2.right_trigger > 0){
                armTempPos = armTarget + 10;
                setTargetArm(armTempPos);
            } else if(gamepad2.left_trigger >0) {
                armTempPos = armTarget - 10;
                setTargetArm(armTempPos);
            }else{
                currentArmState = armState.HOLDING;
            }

            //GamePad 1 buttons

            if (buttonHandler.isPressedOnceRB_1(gamepad1.right_bumper)) { //change to right bumper
                if (changeSpeedPos) {
                    changeSpeedPos = false;
                    changeSpeed = 0.65; //Normal driving speed

                } else {
                    changeSpeed = 0.25; //Slow driving speed
                    changeSpeedPos = true;
                }
            }

            if (gamepad1.dpad_up){
                hangServo.setPosition(0.3);
            }else {
                hangServo.setPosition(0.5);
            }

            if (gamepad1.dpad_down){
                hangMotor.setPower(0.8);
            }else {
                hangMotor.setPower(0.0);
            }


            //GamePad 2 buttons

            if (buttonHandler.isPressedOnceA_2(gamepad2.a)) {
                wristServo.setPosition(0.4);
            }
            if (buttonHandler.isPressedOnceB_2(gamepad2.b)) {
                wristServo.setPosition(0.8);
            }
            if (buttonHandler.isPressedOnceX_2(gamepad2.x)) {
                wristServo.setPosition(0.1);
            }
            if (gamepad2.y) {
                clawServo.setPosition(0.5); // changed from 0.6
            }  else {
                clawServo.setPosition(0.72); //if claw loose adjust + 0.05
            }
            if (gamepad2.right_bumper){
                basketServo.setPosition(0.7);
            }else if (gamepad2.left_bumper){
                basketServo.setPosition(0.7);
            }else {
                basketServo.setPosition(0.3);
            }
        }
    }

    public void updateTelemetry() {
        telemetry.addData("arm State", currentArmState);
        telemetry.addData("target ", armTarget);
        telemetry.addData("time", newTimer.seconds());
        telemetry.addData("jointMotorPos", jointMotor.getCurrentPosition());
        telemetry.addData("slideMotorPos", slideMotor.getCurrentPosition());
        telemetry.addData("wristServo", wristServo.getPosition());
        telemetry.update();
    }

    public void setTargetArm(int target){
        armTarget = target;
        currentArmState = armState.MOVING;
    }
    public void moveArm(int target){
        controller.setPID(p,i,d);
        int jointPos = jointMotor.getCurrentPosition();
        double pid = controller.calculate(jointPos,target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;
        jointMotor.setPower(power);
    }
    public void armFSM(){
        switch (currentArmState){
            case IDLE:
                int currentPos1 = jointMotor.getCurrentPosition();
                moveArm(currentPos1);
                break;
            case HOLDING:
                moveArm(armTarget);
                break;
            case MOVING:
                moveArm(armTarget);
                break; //delete this if it doesnt work
        }
    }
    public void slideFSM(){
        switch (currentSlideState) {
            case IDLE:
                if(gamepad2.left_stick_y != 0){
                    currentSlideState = SlideState.MANUAL;
                }
                
                break;

            case MOVING:
                // Check if the slide has reached the target
                if (!slideMotor.isBusy()) {
                    // If it's done moving, transition to COMPLETED state
                    currentSlideState = SlideState.COMPLETED;
                }
                break;

            case COMPLETED:
                // Stop the motor and set back to RUN_USING_ENCODER for manual control
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor.setPower(0);  // Stop the motor

                // Now transition back to IDLE for future movement
                currentSlideState = SlideState.IDLE;
                break;

            case MANUAL:
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor.setPower(gamepad2.left_stick_y);
                // If no manual control is happening, return to IDLE
                if (Math.abs(gamepad2.left_stick_y) == 0) {
                    currentSlideState = SlideState.IDLE;
                }
                break;
        }
    }
    public void lightsUpdate(){
        if (60 > newTimer.seconds()){ // First 60 seconds is green
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (90 > newTimer.seconds()){ // From 60 to 90 seconds is blue
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (105 > newTimer.seconds()){ // From 90 to 105 seconds is white
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else if (120 > newTimer.seconds()){ // From 105 to 120 seconds is red
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (140 > newTimer.seconds()){ // From 120 to 140 seconds is yellow
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }
}



