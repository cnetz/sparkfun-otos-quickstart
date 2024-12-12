package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp
public class RRTesting  extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 54, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(-36, 0), Math.toRadians(-90))
                        //.splineTo(new Vector2d(-47, 24), Math.PI / 2)
                        // Align for first placement
                       // .lineToYLinearHeading(30, Math.toRadians(-90))
                        //.waitSeconds(1)
                        // Move back to place
                        //.lineToYLinearHeading(40, Math.toRadians(-90))
                        //.strafeTo(new Vector2d(-30, 40))
                        //.strafeToConstantHeading(new Vector2d(-30, 40), Math.toRadians(90))
                       //.strafeToConstantHeading(new Vector2d(-30, 40))
                        .build());
    }
}
