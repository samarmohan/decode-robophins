package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.CLOSE_PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.AutonSpindexer;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;
import org.firstinspires.ftc.teamcode.teleop.Intake;

@Autonomous(name = "BLUE - FAR - 9 Auton")
public class BlueFar9 extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        final double INTAKE_FORWARD_SPEED = 25.0;

        Pose2d z = new Pose2d(0, 0, BLUE_COLLECT_ROTATION);

        Pose2d initialPose = new Pose2d(60, -12, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(50, -12);
        Vector2d collectFirstSetPartOne = new Vector2d(53, -61);
        Vector2d collectFirstSetTransition = new Vector2d(59, -50);
        Vector2d collectFirstSetPartTwo = new Vector2d(63, -62);;
        Vector2d lineUpSecondSet = new Vector2d(34, -25);
        Vector2d collectSecondSet = new Vector2d(34, -62);

        Intake intake = new Intake();
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intake.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap, limelight, "BLUE");
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake, limelight);

        Action action = new ParallelAction(
                turret.setPitchPosition(FAR_PITCH_POSITION),
                turret.setFlywheelRPM(FAR_FLYWHEEL_RPM),
                spindexer.updateSpindexer(),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .stopAndAdd(spindexer.startLimeight())
                        .strafeTo(shooting)
                        .afterTime(0, spindexer.getObelisk())
                        .stopAndAdd(turret.rotateRight(700))
                        .stopAndAdd(turret.switchToAimingPipeline())
                        .stopAndAdd(turret.rotateLeft(500))
                        .afterTime(0, turret.updateLimelightPID())
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.intake())

                        .afterTime(0, spindexer.setOrder(1,1,2))
                        .afterTime(0, spindexer.indexBalls(3))

                        .strafeTo(collectFirstSetPartOne, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(collectFirstSetTransition)
                        .strafeTo(collectFirstSetPartTwo, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.slowIntake())

                        // shoot first
                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.intake())

                        .strafeTo(lineUpSecondSet)
                        .afterTime(0, spindexer.setOrder(2,1,1))
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectSecondSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .stopAndAdd(spindexer.slowIntake())

                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(turret.rotateRight(0))

                        // reset
                        .strafeTo(collectFirstSetTransition, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .stopAndAdd(spindexer.off())
                        .stopAndAdd(turret.setFlywheelRPM(0))
                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        telemetry.addData("Runtime", runtime.seconds());
        Actions.runBlocking(action);
    }
}