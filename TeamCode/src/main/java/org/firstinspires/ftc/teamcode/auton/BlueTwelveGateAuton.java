package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.COLLECT_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.GATE_OPEN_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.SHOOT_WAIT_TIME;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;

@Autonomous(name = "BLUE - 12 - GATE Auton")
public class BlueTwelveGateAuton extends LinearOpMode {
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

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d openGate = new Vector2d(-3, -53);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
fakeAction(),
                turret.setPitchPosition(PITCH_POSITION),
                turret.setFlywheelRPM(FLYWHEEL_RPM),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .waitSeconds(1.5)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION + Math.toRadians(3))
                        .afterTime(0, fakeAction())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .afterTime(0, fakeAction())
                        .strafeTo(collectFirstSet)
                        .waitSeconds(COLLECT_WAIT_TIME)

                        // open gate and shoot
                        .strafeTo(lineUpGate)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, fakeAction())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect second spike and shoot
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .afterTime(0, fakeAction())
                        .strafeTo(collectSecondSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
                        .afterTime(0, fakeAction())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, fakeAction())
                        .waitSeconds(0.3)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, fakeAction())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, fakeAction())
                        .afterTime(0, turret.setFlywheelRPM(0))
                        .afterTime(0, turret.setPitchPosition(0))

                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        telemetry.addData("Runtime", runtime.seconds());
        Actions.runBlocking(action);
    }
    public Action fakeAction() {
        return packet -> {
            // do nothing
            return false;
        };
    }
}