package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.CLOSE_FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.*;

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

@Autonomous(name = "BLUE - FAR - 9 Auton")
public class BlueFar9 extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double BLUE_SHOOT_ROTATION = Math.toRadians(-150);
        final double BLUE_SHOOT_ROTATION_SLIGHT_LEFT = Math.toRadians(-132);
        final double BLUE_SHOOT_ROTATION_SLIGHT_RIGHT = Math.toRadians(-138);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);
        final double BLUE_OBELISK_ROTATION = Math.toRadians(-190);

        Pose2d initialPose = new Pose2d(55, -10, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(60, -10);
        Vector2d collectFirstSetPartOne = new Vector2d(53, -60);
        Vector2d collectFirstSetTransition = new Vector2d(55, -50);
        Vector2d collectFirstSetPartTwo = new Vector2d(59, -60);;
        Vector2d lineUpSecondSet = new Vector2d(35, -20);
        Vector2d collectSecondSet = new Vector2d(35, -60);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap);
        Intake intake = new Intake();
        intake.init(hardwareMap);
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake);

        Action action = new ParallelAction(
                turret.setPitchPosition(PITCH_POSITION),
                turret.setFlywheelRPM(FAR_FLYWHEEL_RPM),
                spindexer.updateSpindexer(),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .stopAndAdd(spindexer.startLimeight())
                        .strafeTo(new Vector2d(50, -10))
                        .afterTime(0, spindexer.getObelisk())
                        .turnTo(BLUE_OBELISK_ROTATION)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .turnTo(BLUE_SHOOT_ROTATION)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike and shoot
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .afterTime(0, spindexer.intake())
                        .afterTime(0, spindexer.setOrder(1,1,2))
                        .afterTime(1, spindexer.index())
                        .afterTime(2, spindexer.index())
                        .strafeTo(collectFirstSetPartOne)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeTo(collectFirstSetTransition)
                        .strafeTo(collectFirstSetPartTwo)
                        .waitSeconds(COLLECT_WAIT_TIME)

                        // shoot first
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, spindexer.outtake())
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        .strafeToSplineHeading(lineUpSecondSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, spindexer.setOrder(2,1,1))
                        .afterTime(0, spindexer.intake())
                        .afterTime(1, spindexer.index())
                        .afterTime(2, spindexer.index())
                        .strafeTo(collectSecondSet)

                        // shoot second
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, spindexer.outtake())
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        .strafeToSplineHeading(collectFirstSetTransition, BLUE_COLLECT_ROTATION)

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