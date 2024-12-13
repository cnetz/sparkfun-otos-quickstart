package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 12.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 58, Math.toRadians(-90)))
                        .splineToConstantHeading(new Vector2d(0, 30), -90) // Place first specimen
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(-32, 38), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-38, 12), -90) // Move towards opposite aliance to push specimen
                        .splineToConstantHeading(new Vector2d(-46, 58), -90) // Push first sample for hp
                        .splineToConstantHeading(new Vector2d(-48, 12), -90) // Go back
                        .splineToConstantHeading(new Vector2d(-56, 58), -90) // Push second sample for hp
                        .splineToConstantHeading(new Vector2d(-60, 12), -90) // Go back
                        .setReversed(true)
                        .lineToY(54)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}