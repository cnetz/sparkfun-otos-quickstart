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
    double P = 0.006, I = 0, D = 0.0001;
    double F = 0.04;
    double ticksInDegrees = 358.466 / 180;
    int threshold = 50;

    private boolean isHolding = false;

    public Slide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(P, I, D);
    }

    public class SlideToPosition implements Action {
        private final double targetPosition;
        private boolean initialized = false;


        public SlideToPosition(int target) {
            this.targetPosition = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                controller.setPID(P, I, D);
                initialized = true;
            }

            int currentPosition = slideMotor.getCurrentPosition();
            double pid = controller.calculate(currentPosition, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * F;
            double power = pid + ff; //double power = Math.max(-1.0, Math.min(1.0, pid + ff));
            slideMotor.setPower(power);

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
            int currentPosition = slideMotor.getCurrentPosition();
            double power = Math.cos(Math.toRadians(currentPosition / ticksInDegrees)) * F;
            slideMotor.setPower(power);
        }
    }

    // RoadRunner Action that is called in any autonomous class to set the slide to a position
    public Action setTargetPosition(double targetPosition) {
        return new SlideToPosition((int) targetPosition);
    }
}
