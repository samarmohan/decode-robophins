package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueFar {
    public static final double SHOOT_WAIT_TIME = 0.5;
    public static final double COLLECT_WAIT_TIME = 0.5;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 13.03)
                .build();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-150);
        final double BLUE_SHOOT_ROTATION_SLIGHT_LEFT = Math.toRadians(-132);
        final double BLUE_SHOOT_ROTATION_SLIGHT_RIGHT = Math.toRadians(-138);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
        final double BLUE_OBELISK_ROTATION = Math.toRadians(-190);

        Pose2d initialPose = new Pose2d(57, -16, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(55, -10);
        Vector2d collectFirstSetPartOne = new Vector2d(53, -57);
        Vector2d collectFirstSetTransition = new Vector2d(55, -50);
        Vector2d collectFirstSetPartTwo = new Vector2d(59, -57);;
        Vector2d lineUpSecondSet = new Vector2d(35, -20);
        Vector2d collectSecondSet = new Vector2d(35, -57);

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                // shoot preset balls
//                .stopAndAdd(spindexer.intake())
//                .stopAndAdd(spindexer.startLimeight())
//                .stopAndAdd(spindexer.getObelisk())
                .waitSeconds(0.5)
                .turnTo(BLUE_OBELISK_ROTATION)
//                .stopAndAdd(spindexer.alignForShooting())
                .waitSeconds(SHOOT_WAIT_TIME)
                .turnTo(BLUE_SHOOT_ROTATION)
//                .stopAndAdd(spindexer.shoot())
                .waitSeconds(SHOOT_WAIT_TIME)
//
                // collect first spike and shoot
                .turnTo(BLUE_COLLECT_ROTATION)
                .strafeTo(collectFirstSetPartOne)
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeTo(collectFirstSetTransition)
                .strafeTo(collectFirstSetPartTwo)
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                .waitSeconds(SHOOT_WAIT_TIME)

                .strafeToSplineHeading(lineUpSecondSet, BLUE_COLLECT_ROTATION)
                .strafeTo(collectSecondSet)
                .waitSeconds(COLLECT_WAIT_TIME)
                .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                .waitSeconds(SHOOT_WAIT_TIME)
                .strafeToSplineHeading(collectFirstSetTransition, BLUE_COLLECT_ROTATION)

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