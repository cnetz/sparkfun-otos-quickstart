package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Joint {
    private DcMotorEx jointMotor;

    private PIDController controller;
    private double P = 0.004, I = 0, D = 0.0002;
    private double F = 0.03;
    private double ticksInDegrees = 285 / 180;
    private int threshold = 10;
    private boolean isHolding = false;

    public Joint(HardwareMap hardwareMap) {
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor");
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDController(P, I, D);
    }

    public class JointToPosition implements Action {
        private final double targetPosition;
        private boolean initialized = false;

        public JointToPosition(int target) {
            this.targetPosition = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                controller.setPID(P, I, D);
                initialized = true;
            }

            int currentPosition = jointMotor.getCurrentPosition();
            double pid = controller.calculate(currentPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * F;
            double power = pid + ff; //double power = Math.max(-1.0, Math.min(1.0, pid + ff));
            jointMotor.setPower(power);

            telemetryPacket.put("slidePos", currentPosition);
            telemetryPacket.put("slideTarget", targetPosition);
            telemetryPacket.put("slidePower", power);

            if (Math.abs(currentPosition - targetPosition) < threshold) {
                isHolding = true;
                return false; // Indicate action is complete
            } else {
                isHolding = false;
                return true; // Continue moving
            }
        }
    }

    // Feedforward PID Controller to counter act gravity when not moving
    public void holdPosition() {
        if (isHolding) {
            int currentPosition = jointMotor.getCurrentPosition();
            double power = Math.cos(Math.toRadians(currentPosition / ticksInDegrees)) * F;
            jointMotor.setPower(power);
        }
    }

    // RoadRunner Action that is called in any autonomous class to set the joint to a position
    public Action setTargetPosition(double targetPosition) {
        return new Joint.JointToPosition((int) targetPosition);
    }
}
