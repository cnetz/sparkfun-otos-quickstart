package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Basket;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Joint;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class SpecimenRRAuto extends OpMode {
    private Slide slide;
    private Joint joint;
    private Basket basket;
    private Wrist wrist;
    private Claw claw;

    private Pose2d beginPose;
    private PinpointDrive drive;
    private TrajectoryActionBuilder first;

    @Override
    public void init() {
        slide = new Slide(hardwareMap);
        joint = new Joint(hardwareMap);
        basket = new Basket(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);

        beginPose = new Pose2d(0, 54, Math.toRadians(-90));
        drive = new PinpointDrive(hardwareMap, beginPose);


        //Sets claw to an inital position to hold specimen
        Actions.runBlocking(claw.setTargetPosition(0.5));
    }

    @Override
    public void init_loop() {


        first = drive.actionBuilder(beginPose)
                .lineToX(-33)
                .stopAndAdd(claw.setTargetPosition(0.5))
                .waitSeconds(5);

    }

    @Override
    public void loop() {
        Actions.runBlocking(
                new SequentialAction(
                    first.build(),
                    slide.setTargetPosition(1000),
                    basket.setTargetPosition(1)
                )
        );


    }
}
