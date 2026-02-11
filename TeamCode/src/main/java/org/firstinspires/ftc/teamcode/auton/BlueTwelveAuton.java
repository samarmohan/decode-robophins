package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.COLLECT_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.SHOOT_WAIT_TIME;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.AutonSpindexer;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;
import org.firstinspires.ftc.teamcode.teleop.Intake;

@Autonomous(name = "BLUE - 12 - NO Gate Auton")
public class BlueTwelveAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        Pose2d initialPose = new Pose2d(-40, -52, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-14, -14);
        Vector2d collectFirstSet = new Vector2d(-12, -50);
        Vector2d lineUpSecondSet = new Vector2d(12, -22);
        Vector2d collectSecondSet = new Vector2d(12, -57);
        Vector2d lineUpThirdSet = new Vector2d(36, -20);
        Vector2d collectThirdSet = new Vector2d(36, -57);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap);
        Intake intake = new Intake();
        intake.init(hardwareMap);
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                spindexer.updateServos(),
                turret.setPitchPosition(PITCH_POSITION),
                turret.setFlywheelRPM(FLYWHEEL_RPM),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .stopAndAdd(spindexer.startIntaking() )
                        .waitSeconds(1.5)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION + Math.toRadians(3))
                        //.stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        //.stopAndAdd(spindexer.startIntaking())
                        .strafeTo(collectFirstSet)
                       // .stopAndAdd(spindexer.indexBall())
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        //.stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

//                        // collect second spike and shoot
//                        .setReversed(true)
//                        .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
//                        .afterTime(0, spindexer.intake())
//                        .strafeTo(collectSecondSet)
//                        .waitSeconds(COLLECT_WAIT_TIME)
//                        .setReversed(true)
//                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
//                        .afterTime(0, spindexer.shoot())
//                        .waitSeconds(SHOOT_WAIT_TIME)
//
//                        // collect third spike and shoot
//                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
//                        .afterTime(0, spindexer.intake())
//                        .waitSeconds(0.3)
//                        .strafeTo(collectThirdSet)
//                        .waitSeconds(COLLECT_WAIT_TIME)
//                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
//                        .afterTime(0, spindexer.shoot())
//                        .waitSeconds(SHOOT_WAIT_TIME)
//
//                        // reset
//                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
//                        .afterTime(0, spindexer.hold())
//                        .afterTime(0, turret.setFlywheelRPM(0))
//                        .afterTime(0, turret.setPitchPosition(0))

                        .build()
        );

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(action);
    }

    public Action fakeAction() {
        return packet -> {
            // do nothing
            return false;
        };
    }
}