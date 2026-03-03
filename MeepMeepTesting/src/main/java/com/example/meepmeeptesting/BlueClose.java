package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueClose {
    public static final double SHOOT_WAIT_TIME = 0.5;
    public static final double COLLECT_WAIT_TIME = 0.5;
    public static final double GATE_OPEN_TIME = 1.0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_SHOOT_ROTATION_SLIGHT_LEFT = Math.toRadians(-132);
        final double BLUE_SHOOT_ROTATION_SLIGHT_RIGHT = Math.toRadians(-138);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
        final double BLUE_OBELISK_ROTATION = Math.toRadians(-200);

        Pose2d initialPose = new Pose2d(-40, -54, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-10, -14);
        Vector2d collectFirstSet = new Vector2d(-10, -50);
        Vector2d lineUpSecondSet = new Vector2d(15, -22);
        Vector2d collectSecondSet = new Vector2d(15, -57);
        Vector2d lineUpThirdSet = new Vector2d(40, -20);
        Vector2d collectThirdSet = new Vector2d(40, -60);

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d openGate = new Vector2d(-3, -53);

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                // shoot preset balls
//                .stopAndAdd(spindexer.intake())
//                .stopAndAdd(spindexer.startLimeight())
                .setReversed(true)
                .strafeToSplineHeading(shooting, BLUE_OBELISK_ROTATION)
//                .stopAndAdd(spindexer.getObelisk())
                .waitSeconds(0.5)
                .turnTo(BLUE_SHOOT_ROTATION)
//                .stopAndAdd(spindexer.alignForShooting())
                .waitSeconds(SHOOT_WAIT_TIME)
//                .stopAndAdd(spindexer.shoot())
                .waitSeconds(SHOOT_WAIT_TIME)
//
                // collect first spike and shoot
                .turnTo(BLUE_COLLECT_ROTATION)
                .strafeTo(collectFirstSet)
//                .stopAndAdd(spindexer.indexBall(1))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(1))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(2))

                // OPEN GATE
                .strafeTo(lineUpGate)
                .strafeTo(openGate)
                .waitSeconds(GATE_OPEN_TIME)

                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
//                .stopAndAdd(spindexer.alignForShooting())
                .waitSeconds(SHOOT_WAIT_TIME)
//                .stopAndAdd(spindexer.shoot())
                .waitSeconds(SHOOT_WAIT_TIME)
                .turnTo(Math.toRadians(-90))

                // collect second spike and shoot
                .setReversed(true)
                .strafeTo(lineUpSecondSet)
                .waitSeconds(0.3)
                .strafeTo(collectSecondSet)
                .waitSeconds(0.3)
//                .stopAndAdd(spindexer.indexBall(1))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(2))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(1))

                .strafeTo(lineUpGate)
                .strafeTo(openGate)
                .waitSeconds(GATE_OPEN_TIME)

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
//                .stopAndAdd(spindexer.alignForShooting())
                .waitSeconds(SHOOT_WAIT_TIME)
//                .stopAndAdd(spindexer.shoot())
                .waitSeconds(SHOOT_WAIT_TIME)

                // collect third spike and shoot
                .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                .waitSeconds(0.3)
                .strafeTo(collectThirdSet)
//                .stopAndAdd(spindexer.indexBall(2))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(1))
                .waitSeconds(COLLECT_WAIT_TIME)
//                .stopAndAdd(spindexer.indexBall(1))
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
//                .stopAndAdd(spindexer.alignForShooting())
                .waitSeconds(0.3)
//                .stopAndAdd(spindexer.shoot())
                .waitSeconds(SHOOT_WAIT_TIME)

                // reset
                .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
//                .afterTime(0, spindexer.off())
//                .afterTime(0, turret.setFlywheelRPM(0))
//                .afterTime(0, turret.setPitchPosition(0))
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