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
import org.firstinspires.ftc.teamcode.auton.parts.AutonSpindexer;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;
import org.firstinspires.ftc.teamcode.teleop.Intake;

@Autonomous(name = "BLUE - CLOSE - 12 - GATE Auton")
public class BlueTwelveGateAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_SHOOT_ROTATION_SLIGHT_LEFT = Math.toRadians(-132);
        final double BLUE_SHOOT_ROTATION_SLIGHT_RIGHT = Math.toRadians(-138);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
        final double BLUE_OBELISK_ROTATION = Math.toRadians(-200);

        Pose2d initialPose = new Pose2d(-40, -52, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-10, -14);
        Vector2d collectFirstSet = new Vector2d(-10, -50);
        Vector2d lineUpSecondSet = new Vector2d(15, -22);
        Vector2d collectSecondSet = new Vector2d(15, -57);
        Vector2d lineUpThirdSet = new Vector2d(40, -20);
        Vector2d collectThirdSet = new Vector2d(40, -60);

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d openGate = new Vector2d(-3, -53);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, BLUE_COLLECT_ROTATION));
        AutonTurret turret = new AutonTurret(hardwareMap);
        Intake intake = new Intake();
        intake.init(hardwareMap);
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                turret.setPitchPosition(PITCH_POSITION),
                turret.setFlywheelRPM(FLYWHEEL_RPM),
                drive.actionBuilder(new Pose2d(0, 0, BLUE_COLLECT_ROTATION))
                .stopAndAdd(spindexer.intake())
                .stopAndAdd(spindexer.startLimeight())
                        .strafeTo(initialPose.position)
                        .waitSeconds(5)
                        /*
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_OBELISK_ROTATION)
                        .stopAndAdd(spindexer.getObelisk())
                        .waitSeconds(0.5)
                        .turnTo(BLUE_SHOOT_ROTATION)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .strafeTo(collectFirstSet)
                        .stopAndAdd(spindexer.index(1))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(2))

                        // OPEN GATE
                        .strafeTo(lineUpGate)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)

                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .turnTo(Math.toRadians(-90))

                        // collect second spike and shoot
                        .setReversed(true)
                        .strafeTo(lineUpSecondSet)
                        .waitSeconds(0.3)
                        .strafeTo(collectSecondSet)
                        .waitSeconds(0.3)
                        .stopAndAdd(spindexer.index(1))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(2))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))

                        .strafeTo(lineUpGate)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)

                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                        .waitSeconds(0.3)
                        .strafeTo(collectThirdSet)
                        .stopAndAdd(spindexer.index(2))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.index(1))
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .stopAndAdd(spindexer.alignForShooting())
                        .waitSeconds(0.3)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
 */

                        // reset
                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
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