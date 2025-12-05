package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.Intake;
import org.firstinspires.ftc.teamcode.auton.parts.Turret;


@Autonomous(name = "BlueCloseAuton")
public class BlueCloseAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -53, Math.toRadians(-90));
        Vector2d shooting = new Vector2d(-12, -12);
        Vector2d collectFirstSet = new Vector2d(-12, -53);
        Vector2d lineUpSecondSet = new Vector2d(12, -24);
        Vector2d collectSecondSet = new Vector2d(12, -60);
        Vector2d lineUpThirdSet = new Vector2d(36, -24);
        Vector2d collectThirdSet = new Vector2d(36, -60);

        final double SHOOT_WAIT_TIME = 3.0;
        final double COLLECT_WAIT_TIME = 0.3;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                intake.intakeIn(),
                turret.setFlywheelRPM(),
                turret.setPitchPosition(),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .strafeToSplineHeading(shooting, Math.toRadians(-135))
                        .afterTime(0.1, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect first spike and shoot
                        .strafeToSplineHeading(collectFirstSet, Math.toRadians(-90))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, Math.toRadians(-135))
                        .afterTime(0.1, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .afterTime(0, intake.intakeIn())

                        // collect second spike and shoot
                        .strafeToSplineHeading(lineUpSecondSet, Math.toRadians(-90))
                        .strafeTo(collectSecondSet)
                        .strafeTo(new Vector2d(12, -50))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, Math.toRadians(-135))
                        .afterTime(0.1, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect third spike and shoot
//                        .strafeToSplineHeading(lineUpThirdSet, Math.toRadians(-90))
//                        .afterTime(0, intake.intakeIn())
//                        .strafeTo(collectThirdSet)
//                        .waitSeconds(COLLECT_WAIT_TIME)
//                        .strafeToSplineHeading(shooting, Math.toRadians(-135))
//                        .afterTime(0.1, intake.intakeShoot())
//                        .waitSeconds(SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, Math.toRadians(-90))
                        .afterTime(0, intake.intakeOff())

                        .build()
        );

        waitForStart();
        telemetry.update();
        while (opModeIsActive()) {
            Actions.runBlocking(
                    action
            );
        }

    }
}