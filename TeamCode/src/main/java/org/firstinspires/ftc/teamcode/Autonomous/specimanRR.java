package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mechanisms.Basket;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Joint;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class specimanRR extends OpMode {
    private Slide slide;
    private Joint joint;
    private Basket basket;
    private Wrist wrist;
    private Claw claw;
    private Pose2d beginPosition;
    private PinpointDrive drive;
    private TrajectoryActionBuilder first;
    @Override
    public void init() {
        slide = new Slide(hardwareMap);
        joint = new Joint(hardwareMap);
        basket = new Basket(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);

        beginPosition = new Pose2d(0,54,Math.toRadians(-90));
        drive = new PinpointDrive(hardwareMap,beginPosition);

        Actions.runBlocking(claw.setTargetPosition(0.5));
    }

    @Override
    public void init_loop(){
        first = drive.actionBuilder(beginPosition)
                .splineToConstantHeading(new Vector2d(0,30), -90)
                //.waitSeconds(5)
                ;
    }
    @Override
    public void loop(){
        slide.holdingPosition();
        joint.holdingPosition();
    }
    @Override
    public void start() {
        Actions.runBlocking(
                new SequentialAction(
                        first.build(),
                        basket.setTargetPosition(1)
                )
        );
    }
}
