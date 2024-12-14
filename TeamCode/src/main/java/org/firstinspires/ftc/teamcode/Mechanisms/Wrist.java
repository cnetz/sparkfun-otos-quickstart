package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class,"wristServo");
    }

    public class WristToPosition implements Action {
        private final int targetPosition;
        public WristToPosition(int target) {this.targetPosition = target;}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(targetPosition);
            return false;
        }
    }
    public Action setTargetPosition(double targetPosition) {
        return new Wrist.WristToPosition((int) targetPosition);
    }
}
