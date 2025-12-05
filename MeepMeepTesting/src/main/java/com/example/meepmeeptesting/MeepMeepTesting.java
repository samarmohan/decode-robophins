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
    private static final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
    private static final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
    public static final double SHOOT_WAIT_TIME = 2.8;
    public static final double COLLECT_WAIT_TIME = 0.2;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();

        Pose2d initialPose = new Pose2d(-38, -53, Math.toRadians(-90));
        Vector2d shooting = new Vector2d(-12, -12);
        Vector2d collectFirstSet = new Vector2d(-12, -53);
        Vector2d lineUpSecondSet = new Vector2d(12, -24);
        Vector2d collectSecondSet = new Vector2d(12, -60);
        Vector2d lineUpThirdSet = new Vector2d(36, -24);
        Vector2d collectThirdSet = new Vector2d(36, -60);

        Action action = new Action() {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                        // shoot preset balls
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0.1, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(action)

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFirstSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0.1, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .afterTime(0, action)

                        // collect second spike and shoot
                        .strafeToSplineHeading(lineUpSecondSet, BLUE_COLLECT_ROTATION)
                        .strafeTo(collectSecondSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        //.strafeTo(new Vector2d(12, -50))
                        .splineToConstantHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0.1, action)
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, action)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0.1, action)
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, action)
                        .afterTime(0, action)
                        .afterTime(0, action)

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}