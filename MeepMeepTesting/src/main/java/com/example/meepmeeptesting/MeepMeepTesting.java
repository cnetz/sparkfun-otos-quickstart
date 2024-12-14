package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.PI, Math.PI, 12.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 58, Math.PI / 2))
                //.setReversed(true)
                //.splineToLinearHeading(new Pose2d(0, 30, Math.PI / 2), - 90)
                //.waitSeconds(4)
                        //.setReversed(true)
                //.splineTo(new Vector2d(-32,38),-Math.PI / 2 )
                //.splineToConstantHeading(new Vector2d(-38,12), -90)
               //         //.setReversed(true)
               // .splineToConstantHeading(new Vector2d(-46,56),-90)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, 28, Math.PI / 2), -90)
                .waitSeconds(4)
                .splineToConstantHeading(new Vector2d(-32,38),-90)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}