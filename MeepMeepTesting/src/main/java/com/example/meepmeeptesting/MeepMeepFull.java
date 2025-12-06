package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;


public class MeepMeepFull {
    private static final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
    private static final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
    public static final double SHOOT_WAIT_TIME = 2.4;
    public static final double COLLECT_WAIT_TIME = 0.001;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        Pose2d initialPose = new Pose2d(-38, -53, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-12, -12);
        Vector2d collectFirstSet = new Vector2d(-12, -50);
        Vector2d lineUpSecondSet = new Vector2d(12, -24);
        Vector2d collectSecondSet = new Vector2d(12, -57);
        Vector2d lineUpThirdSet = new Vector2d(36, -24);
        Vector2d collectThirdSet = new Vector2d(36, -57);

        Vector2d lineUpFourthSet = new Vector2d(60, -24);
        Vector2d collectFourthSet = new Vector2d(60, -57);

        Action action = new Action() {
            @Override
            public boolean run(@NotNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                        // shoot preset balls
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(action)

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFirstSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(action)

                        // collect second spike and shoot
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectSecondSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), Math.toRadians(135))
                        .afterTime(0, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(action)

                        // collect third spike and shoot
                        .splineToSplineHeading(new Pose2d(lineUpThirdSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, action)
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(action)

                         //fourth
                        .splineToSplineHeading(new Pose2d(lineUpFourthSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFourthSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, action)
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