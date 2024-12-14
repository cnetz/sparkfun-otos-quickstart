package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Basket;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Joint;
import org.firstinspires.ftc.teamcode.Mechanisms.Slide;
import org.firstinspires.ftc.teamcode.Mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous
public class specimanRR extends OpMode {
    private Slide slide;
    private Joint joint;
    private Basket basket;
    private Wrist wrist;
    private Claw claw;
    private Pose2d beginPosition;
    private PinpointDrive drive;
    private TrajectoryActionBuilder first;
    private TrajectoryActionBuilder second;

    @Override
    public void init() {
        slide = new Slide(hardwareMap);
        joint = new Joint(hardwareMap);
        basket = new Basket(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);

        beginPosition = new Pose2d(0,58,Math.PI / 2);
        drive = new PinpointDrive(hardwareMap,beginPosition);

        Actions.runBlocking(claw.setTargetPosition(0.85));
        Actions.runBlocking(wrist.setTargetPosition(0.90));
        //Actions.runBlocking(basket.setTargetPosition(0.3));
    }

    @Override
    public void init_loop(){
        first = drive.actionBuilder(beginPosition)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, 28, Math.PI / 2), -90)
                .waitSeconds(4)
                .splineToConstantHeading(new Vector2d(-32,38),-90)
                ;

//        second = drive.actionBuilder()
//                .splineTo(new Vector2d(-32,38),-Math.PI / 2 )
//                //.splineToConstantHeading(new Vector2d(-38,12), -90)
//                //.splineToConstantHeading(new Vector2d(-46,56),-90)
//                ;

        second = first.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-32,38),-90)
                ;


        updateTelemetry();
    }
    @Override
    public void loop(){
        slide.holdingPosition();
        joint.holdingPosition();

        updateTelemetry();

    }
    @Override
    public void start() {
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                first.build(),
                                slide.setTargetPosition(3000),
                                joint.setTargetPosition(1500),
                                wrist.setTargetPosition(0.35)
                        )
                        claw.setTargetPosition(0.55)
                        //second.build()

                )
        );
    }

    public void updateTelemetry() {
        telemetry.addData("Basket Position", basket.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Joint Position", joint.getCurrentPosition());
        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.update();
    }
}
