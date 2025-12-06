package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;


public class MeepMeepSamples {
    private static final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
    private static final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
    public static final double SHOOT_WAIT_TIME = 2.8;
    public static final double COLLECT_WAIT_TIME = 0.1;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0,0,0))
                    .waitSeconds(1)
                        .strafeToSplineHeading(new Vector2d(20, 20), Math.toRadians(90))
                        .strafeTo(new Vector2d(0,0))
                        .turnTo(0)
                        .waitSeconds(1)
                        .splineTo(new Vector2d(20, 20), Math.toRadians(90))
                        .strafeTo(new Vector2d(0,0))
                        .turnTo(0)
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(20, 20, Math.toRadians(0)), Math.toRadians(0))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}