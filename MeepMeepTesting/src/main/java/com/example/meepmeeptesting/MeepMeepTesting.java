package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d initialPose = new Pose2d(-38, -53, Math.toRadians(-90));
        Vector2d shooting = new Vector2d(-12, -12);
        Vector2d collectFirstSet = new Vector2d(-12, -53);
        Vector2d lineUpSecondSet = new Vector2d(12, -24);
        Vector2d collectSecondSet = new Vector2d(12, -60);
        Vector2d lineUpThirdSet = new Vector2d(36, -24);
        Vector2d collectThirdSet = new Vector2d(36, -60);

        final double SHOOT_WAIT_TIME = 3.0;
        final double COLLECT_WAIT_TIME = 0.3;

        Action action = new Action() {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .strafeToSplineHeading(shooting, Math.toRadians(-135))
                .afterTime(0.1, action)

                .waitSeconds(SHOOT_WAIT_TIME)
                .stopAndAdd(action)
                .strafeToSplineHeading(collectFirstSet, Math.toRadians(-90))

                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, Math.toRadians(-135))
                .afterTime(0.1, action)

                .waitSeconds(SHOOT_WAIT_TIME)
                .afterTime(0, action)
                .strafeToSplineHeading(lineUpSecondSet, Math.toRadians(-90))

                .strafeTo(collectSecondSet)
                        .strafeTo(new Vector2d(12, -50))
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, Math.toRadians(-135))
                .afterTime(0.1, action)

                .waitSeconds(SHOOT_WAIT_TIME)
                .strafeToSplineHeading(lineUpThirdSet, Math.toRadians(-90))
                .afterTime(0, action)

                .strafeTo(collectThirdSet)
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, Math.toRadians(-135))
                .afterTime(0.1, action)

                .waitSeconds(SHOOT_WAIT_TIME)
                .strafeToSplineHeading(collectFirstSet, Math.toRadians(-90))
                .afterTime(0, action)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}