package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.COLLECT_WAIT_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.FAR_FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.FAR_PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.SHOOT_WAIT_TIME;

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

@Autonomous(name = "BLUE - FAR - DOUBLE WALL (10138) - 9 Auton")
public class BlueFar9DoubleWall extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);

        final double INTAKE_FORWARD_SPEED = 25.0;

        Pose2d z = new Pose2d(0, 0, BLUE_COLLECT_ROTATION);

        Pose2d initialPose = new Pose2d(60, -12, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(50, -12);
        Vector2d collectFirstSetPartOne = new Vector2d(55, -60);
        Vector2d collectFirstSetTransition = new Vector2d(63, -45);
        Vector2d collectFirstSetPartTwo = new Vector2d(63, -60);;
        Vector2d lineUpSecondSet = new Vector2d(34, -25);
        Vector2d collectSecondSet = new Vector2d(34, -60);

        Vector2d park = new Vector2d(45, -45);

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
                        // Setup + Shoot Preload
                        .stopAndAdd(spindexer.startLimeight())
                        .stopAndAdd(spindexer.intake())
                        .strafeTo(shooting)
                        .afterTime(0, spindexer.getObelisk())
                        .stopAndAdd(turret.rotateRight(700))
                        .stopAndAdd(turret.startAutoAim())
                        .stopAndAdd(turret.rotateLeft(500))
                        .afterTime(0, turret.updateLimelightPID())
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME+0.5)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // Collect Wall Balls
                        .afterTime(0, spindexer.setOrder(1,2,1))
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectFirstSetPartOne, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(collectFirstSetTransition)
                        .strafeTo(collectFirstSetPartTwo, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.slowIntake())

                        // Shoot Wall Balls
                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.intake())

                        // Collect Wall Balls
                        .afterTime(0, spindexer.setOrder(1,2,1))
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectFirstSetPartOne, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(collectFirstSetTransition)
                        .strafeTo(collectFirstSetPartTwo, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .stopAndAdd(spindexer.slowIntake())

                        // Shoot Wall Balls PART 2
                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(turret.stopAutoAim())
                        .stopAndAdd(turret.rotateLeft(10))
                        .stopAndAdd(spindexer.outtake())

                        // Power Down
                        .strafeTo(park, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        telemetry.addData("Runtime", runtime.seconds());
        Actions.runBlocking(action);
    }
}