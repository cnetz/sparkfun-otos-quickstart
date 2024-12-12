package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
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

    public Slide(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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

            int slidePos = slideMotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * F;
            double power = pid + ff; //double power = Math.max(-1.0, Math.min(1.0, pid + ff));
            slideMotor.setPower(power);

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

    public Action slideToPosition(double targetPosition) {
        return new SlideToPosition((int) targetPosition);
    }
}
