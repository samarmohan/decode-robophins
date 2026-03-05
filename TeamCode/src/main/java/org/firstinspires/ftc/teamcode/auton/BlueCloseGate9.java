package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.CLOSE_FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.GATE_OPEN_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.SHOOT_WAIT_TIME;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.AutonSpindexer;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;
import org.firstinspires.ftc.teamcode.teleop.Intake;

@Autonomous(name = "BLUE - CLOSE - 9 - 1 GATE Auton")
public class BlueCloseGate9 extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        final double INTAKE_FORWARD_SPEED = 25;

        Pose2d initialPose = new Pose2d(-40, -54, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-12, -20);
        Vector2d collectFirstSet = new Vector2d(-12, -54);
        Vector2d lineUpSecondSet = new Vector2d(12, -22);
        Vector2d collectSecondSet = new Vector2d(12, -60);
        Vector2d lineUpThirdSet = new Vector2d(38, -20);
        Vector2d collectThirdSet = new Vector2d(38, -60);

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d lineUpGateTwo = new Vector2d(-3, -50);
        Vector2d openGate = new Vector2d(-3, -53);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap);
        Intake intake = new Intake();
        intake.init(hardwareMap);
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake);

        Action action = new ParallelAction(
                turret.setPitchPosition(PITCH_POSITION),
                turret.setFlywheelRPM(CLOSE_FLYWHEEL_RPM),
                spindexer.updateSpindexer(),
                drive.actionBuilder(initialPose)
                        .stopAndAdd(spindexer.startLimeight())
                        .afterTime(0, turret.setRotationPosition(750))
                        .setReversed(true)
                        .afterTime(0, spindexer.getObelisk())
                        .strafeTo(shooting)
                        .afterTime(0, turret.setRotationPosition(450))
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike and shoot
                        .afterTime(0, spindexer.setOrder(1,1,2))
                        .afterTime(0, spindexer.intake())
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectFirstSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))

                        // open gate first time
                        .strafeTo(lineUpGate)
                        .afterTime(0, spindexer.outtake())
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)

                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect second spike and shoot
                        .setReversed(true)
                        .strafeTo(lineUpSecondSet)
                        .afterTime(0, spindexer.setOrder(1,2,1))
                        .afterTime(0, spindexer.intake())
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectSecondSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))

                        /*
                        // open gate second time
                        .setReversed(true)
                        .splineToConstantHeading(lineUpGate, Math.toRadians(-90))
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)
                        */

                        .setReversed(true)
                        .strafeTo(shooting)
                        .afterTime(0, spindexer.outtake())
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        /*
                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(collectThirdSet)
                        .stopAndAdd(spindexer.index(2))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0.2, spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                         */

                        // reset
                        .strafeTo(collectFirstSet)
                        .afterTime(0, spindexer.off())
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
}