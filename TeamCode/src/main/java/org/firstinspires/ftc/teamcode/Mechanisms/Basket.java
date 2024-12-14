package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket {
    private Servo basket;
    public Basket(HardwareMap hardwareMap) {

        basket = hardwareMap.get(Servo.class,"basketServo");
        //basket.setDirection(Servo.Direction.REVERSE);
    }

    public class BasketToPosition implements Action {
        private final double targetPosition;
        public BasketToPosition(double target) {this.targetPosition = target;}

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            basket.setPosition(targetPosition);


            return false;
        }
    }
    public Action setTargetPosition(double targetPosition) {
        return new Basket.BasketToPosition(targetPosition);
    }

    public double getPosition() {
        return basket.getPosition();
    }
}
