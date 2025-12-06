package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.Constants;
import org.firstinspires.ftc.teamcode.auton.parts.Intake;
import org.firstinspires.ftc.teamcode.auton.parts.Turret;


@Autonomous(name = "FifteenBlueCloseAuton")
public class FifteenBlueCloseAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                intake.intakeIn(),
                turret.setFlywheelRPM(Constants.FLYWHEEL_RPM),
                turret.setPitchPosition(Constants.PITCH_POSITION),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(Constants.SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFirstSet)
                        .waitSeconds(Constants.COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(Constants.SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect second spike and shoot
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectSecondSet)
                        .waitSeconds(Constants.COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), Math.toRadians(135))
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(Constants.SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect third spike and shoot
                        .splineToSplineHeading(new Pose2d(lineUpThirdSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(Constants.COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(Constants.SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // fourth
                        .splineToSplineHeading(new Pose2d(lineUpFourthSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFourthSet)
                        .waitSeconds(Constants.COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(Constants.SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, intake.intakeOff())
                        .afterTime(0, turret.setFlywheelRPM(0))
                        .afterTime(0, turret.setPitchPosition(0))

                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        Actions.runBlocking(action);
    }
}