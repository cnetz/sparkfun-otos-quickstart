package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    public class ClawToPosition implements Action {
        private final int targetPosition;

        public ClawToPosition(int target) {
            this.targetPosition = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(targetPosition);
            return false;
        }
    }

    public Action setTargetPosition(double targetPosition) {
        return new Claw.ClawToPosition((int) targetPosition);
    }
}
