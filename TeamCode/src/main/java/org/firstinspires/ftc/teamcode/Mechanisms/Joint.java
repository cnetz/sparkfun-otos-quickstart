package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Joint {

    private DcMotorEx jointMotor;
    private PIDController controller;
    double p = 0.004, i = 0, d = 0.0002;
    double f = 0.03;
    double ticksInDegrees = 285 / 180;
    int threshold = 5;
    private boolean isHolding = false;
    public Joint(HardwareMap hardwareMap){
        jointMotor = hardwareMap.get(DcMotorEx.class,"jointMotor");
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDController(p,i,d);
    }
    public class JointToPosition implements Action {
        private final double targetPosition;
        private boolean initialized = false;
        public JointToPosition(int target){
            this.targetPosition = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                controller.setPID(p, i, d);
                initialized = true;
            }
            int currentPosition = jointMotor.getCurrentPosition();
            double pid = controller.calculate(currentPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;
            double power = pid + ff; //double power = Math.min(1.0, Math.min(1.0, pid + ff));
            jointMotor.setPower(power);
            telemetryPacket.put("jointPos", currentPosition);
            telemetryPacket.put("jointPower", power);
            if (Math.abs(currentPosition - targetPosition) < threshold) {
                isHolding = true;
                return false; // indacate action is complete
            } else {
                isHolding = false;
                return true; // continue moving
            }
        }


    }
    //feedforward pid clntroller to counter act gravity when not moving
    public void holdingPosition(){
        if (isHolding) {
            int currentPosition = jointMotor.getCurrentPosition();
            double power = Math.cos(Math.toRadians(currentPosition / ticksInDegrees)) * f;
            jointMotor.setPower(power);
        }
    }
    //roadRunner Actions that is called in any autonamous class to set the slide to a position
    public Action setTargetPosition(double targetPosition){
        return new JointToPosition((int) targetPosition);
    }

    public double getCurrentPosition() {
        return jointMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return jointMotor.getTargetPosition();
    }

    public double getPower() {
        return jointMotor.getPower();
    }
}
