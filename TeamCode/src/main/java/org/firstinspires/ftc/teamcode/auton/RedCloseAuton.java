package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.Constants.COLLECT_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.Constants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.Constants.PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.Constants.SHOOT_WAIT_TIME;

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


@Autonomous(name = "RedCloseAuton")
public class RedCloseAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        final double RED_SHOOT_ROTATION = Math.toRadians(135);
        final double RED_COLLECT_ROTATION = Math.toRadians(90);

        Pose2d initialPose = new Pose2d(-40, 53, RED_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-12, 12);
        Vector2d collectFirstSet = new Vector2d(-12, 50);
        Vector2d lineUpSecondSet = new Vector2d(12, 24);
        Vector2d collectSecondSet = new Vector2d(12, 57);
        Vector2d lineUpThirdSet = new Vector2d(36, 24);
        Vector2d collectThirdSet = new Vector2d(36, 57);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                intake.intakeIn(),
                turret.setFlywheelRPM(FLYWHEEL_RPM),
                turret.setPitchPosition(PITCH_POSITION),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .strafeToSplineHeading(shooting, RED_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect first spike and shoot
                        .turnTo(RED_COLLECT_ROTATION)
                        .strafeTo(collectFirstSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, RED_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect second spike and shoot
                        .strafeToSplineHeading(lineUpSecondSet, RED_COLLECT_ROTATION)
                        .strafeTo(collectSecondSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(new Vector2d(12, 50))
                        .strafeToSplineHeading(shooting, RED_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(intake.intakeIn())

                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, RED_COLLECT_ROTATION)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, RED_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, RED_COLLECT_ROTATION)
                        .afterTime(0, intake.intakeOff())
                        .afterTime(0, turret.setFlywheelRPM(0))
                        .afterTime(0, turret.setPitchPosition(0))

                        .build()
        );

        waitForStart();
        telemetry.update();
            Actions.runBlocking(action);

    }
}