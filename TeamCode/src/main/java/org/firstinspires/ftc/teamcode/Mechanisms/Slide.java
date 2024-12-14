package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {

    private DcMotorEx slideMotor;
    private PIDController controller;
    double p = 0.006, i = 0, d = 0.0001;
    double f = 0.04;
    double ticksInDegrees = 358.466 / 180;
    int threshold = 50;
    private boolean isHolding = false;
    public Slide(HardwareMap hardwareMap){
        slideMotor = hardwareMap.get(DcMotorEx.class,"SlideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(p,i,d);
    }
    public class SlideToPosition implements Action {
        private final double targetPosition;
        private boolean intitialized = false;
        public SlideToPosition(int target){
            this.targetPosition = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!intitialized) {
                controller.setPID(p, i, d);
                intitialized = true;
            }
            int currentPositoin = slideMotor.getCurrentPosition();
            double pid = controller.calculate(currentPositoin, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
            double power = pid + ff;//double power = Math.min(1.0, Math.min(1.0, pid + ff));
            slideMotor.setPower(power);
            telemetryPacket.put("slidePos", currentPositoin);
            telemetryPacket.put("slidePower", power);
            if (Math.abs(currentPositoin - targetPosition) < threshold) {
                isHolding = true;
                return false;//indacate action is complete
            } else {
                isHolding = false;
                return true; //continue moving
            }
        }


    }
    //feedforward pid clntroller to counter act gravity when not moving
    public void holdingPosition(){
        if (isHolding) {
            int currentPosition = slideMotor.getCurrentPosition();
            double power = Math.cos(Math.toRadians(currentPosition / ticksInDegrees)) * f;
            slideMotor.setPower(power);
        }
    }
    //roadRunner Actions that is called in any autonamous class to set the slide to a position
    public Action setTargetPosition(double targetPositoin){
        return new SlideToPosition((int) targetPositoin);
    }
}
