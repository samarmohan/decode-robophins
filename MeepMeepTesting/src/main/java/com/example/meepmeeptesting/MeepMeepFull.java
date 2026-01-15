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
    public static final double SHOOT_WAIT_TIME = 1.0;
    public static final double COLLECT_WAIT_TIME = 0.001;
    public static final double GATE_OPEN_TIME = 1.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        Pose2d initialPose = new Pose2d(-40, -52, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-14, -14);
        Vector2d collectFirstSet = new Vector2d(-12, -50);
        Vector2d lineUpSecondSet = new Vector2d(12, -22);
        Vector2d collectSecondSet = new Vector2d(12, -57);
        Vector2d lineUpThirdSet = new Vector2d(36, -20);
        Vector2d collectThirdSet = new Vector2d(36, -57);
        Vector2d lineUpFourthSet = new Vector2d(60, -24);
        Vector2d collectFourthSet = new Vector2d(60, -57);

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d openGate = new Vector2d(-3, -53);

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                // shoot preset balls
                .setReversed(true)
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                .afterTime(0, action())
                .waitSeconds(SHOOT_WAIT_TIME)

                // collect first spike
                .turnTo(BLUE_COLLECT_ROTATION)
                .afterTime(0, action())
                .strafeTo(collectFirstSet)
                .waitSeconds(COLLECT_WAIT_TIME)

                // open gate and shoot
                .strafeTo(lineUpGate)
                .strafeTo(openGate)
                .waitSeconds(GATE_OPEN_TIME)
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                .afterTime(0, action())
                .waitSeconds(SHOOT_WAIT_TIME)

                // collect second spike and shoot
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                .afterTime(0, action())
                .strafeTo(collectSecondSet)
                .waitSeconds(COLLECT_WAIT_TIME)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
                .afterTime(0, action())
                .waitSeconds(SHOOT_WAIT_TIME)

                // collect third spike and shoot
                .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                .strafeTo(collectThirdSet)
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                .afterTime(0, action())
                .waitSeconds(SHOOT_WAIT_TIME)

//                // fourth
//                .splineToSplineHeading(new Pose2d(lineUpFourthSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
//                .afterTime(0, action())
//                .strafeTo(collectFourthSet)
//                .waitSeconds(COLLECT_WAIT_TIME+0.5)
//                .setReversed(true)
//                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
//                .afterTime(0, action())
//                .waitSeconds(SHOOT_WAIT_TIME)

                // reset
                .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
                .afterTime(0, action())
                .afterTime(0, action())
                .afterTime(0, action())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
    public static Action action() {
        return packet -> {
            packet.put("Hey", "hey");
            System.out.println("hey");
            return false;
        };
    }
}