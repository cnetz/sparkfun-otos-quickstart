package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Joint {
    private DcMotorEx jointMotor;

    private PIDController controller;
    double P = 0.004, I = 0, D = 0.0002;
    double F = 0.03;
    double ticksInDegrees = 285 / 180;
    int threshold = 10;

    public Joint(HardwareMap hardwareMap) {
        jointMotor = hardwareMap.get(DcMotorEx.class, "jointMotor");
        jointMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        jointMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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

            int slidePos = jointMotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * F;
            double power = pid + ff; //double power = Math.max(-1.0, Math.min(1.0, pid + ff));
            jointMotor.setPower(power);

            telemetryPacket.put("slidePos", slidePos);
            telemetryPacket.put("slideTarget", targetPosition);
            telemetryPacket.put("slidePower", power);

            if (Math.abs(slidePos - targetPosition) < threshold) {
                return false; // Indicate action is complete
            } else {
                return true; // Continue moving
            }
        }
    }

    public Action setTargetPosition(double targetPosition) {
        return new Joint.JointToPosition((int) targetPosition);
    }
}
