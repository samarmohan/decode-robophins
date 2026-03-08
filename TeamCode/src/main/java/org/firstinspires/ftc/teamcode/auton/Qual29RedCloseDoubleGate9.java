package org.firstinspires.ftc.teamcode.auton;


import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.CLOSE_FLYWHEEL_RPM;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.CLOSE_PITCH_POSITION;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.GATE_OPEN_TIME;
import static org.firstinspires.ftc.teamcode.auton.parts.AutonConstants.SHOOT_WAIT_TIME;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.AutonSpindexer;
import org.firstinspires.ftc.teamcode.auton.parts.AutonTurret;
import org.firstinspires.ftc.teamcode.teleop.Intake;

@Disabled
@Autonomous(name = "QUAL #29 - RED - CLOSE - DOUBLE GATE - 9 Auton")
public class Qual29RedCloseDoubleGate9 extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        final double RED_COLLECT_ROTATION = Math.toRadians(90);

        final double INTAKE_FORWARD_SPEED = 25.0;

        Pose2d initialPose = new Pose2d(-40, 54, RED_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-10, 20);
        Vector2d collectFirstSet = new Vector2d(-12, 52);
        Vector2d lineUpSecondSet = new Vector2d(15, 25);
        Vector2d collectSecondSet = new Vector2d(15, 62);
        Vector2d lineUpThirdSet = new Vector2d(34, 25);
        Vector2d collectThirdSet = new Vector2d(34, 62);

        Vector2d lineUpGate = new Vector2d(-3, 45);
        Vector2d secondGateTransition = new Vector2d(15, 40);
        Vector2d lineUpGateTwo = new Vector2d(-3, 40);
        Vector2d openGate = new Vector2d(-3, 53);

        Intake intake = new Intake();
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intake.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AutonTurret turret = new AutonTurret(hardwareMap, limelight, "RED");
        AutonSpindexer spindexer = new AutonSpindexer(hardwareMap, intake, limelight);

        Action action = new ParallelAction(
                turret.setPitchPosition(CLOSE_PITCH_POSITION),
                turret.setFlywheelRPM(CLOSE_FLYWHEEL_RPM),
                spindexer.updateSpindexer(),
                drive.actionBuilder(initialPose)
                        // Setup + Shoot Preload
                        .stopAndAdd(spindexer.startLimeight())
                        .stopAndAdd(spindexer.intake())
                        .strafeTo(shooting)
                        .afterTime(0, spindexer.getObelisk())
                        .stopAndAdd(turret.rotateLeft(-800))
                        .stopAndAdd(turret.startAutoAim())
                        .stopAndAdd(turret.rotateRight(-400))
                        .afterTime(0, turret.updateLimelightPID())
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME+0.3)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.intake())

                        // Collect First Spike
                        .afterTime(0, spindexer.setOrder(1,1,2))
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectFirstSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .stopAndAdd(spindexer.slowIntake())

                        // Open Gate
                        .strafeTo(lineUpGate)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)

                        // Shoot First Spike
                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME+0.2)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(spindexer.intake())

                        // Collect Second Spike
                        .strafeTo(lineUpSecondSet)
                        .afterTime(0, spindexer.setOrder(1,2,1))
                        .afterTime(0, spindexer.indexBalls(3))
                        .strafeTo(collectSecondSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .stopAndAdd(spindexer.slowIntake())

                        // open gate second time
                        .strafeTo(secondGateTransition)
                        //.strafeTo(lineUpGateTwo)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)

                        // Shoot Second Spike
                        .strafeTo(shooting)
                        .stopAndAdd(spindexer.align())
                        .waitSeconds(SHOOT_WAIT_TIME+0.2)
                        .stopAndAdd(spindexer.shoot())
                        .waitSeconds(SHOOT_WAIT_TIME)
                        .stopAndAdd(turret.stopAutoAim())
                        .stopAndAdd(turret.rotateRight(10))
                        .stopAndAdd(spindexer.outtake())

                        // Power Down
                        .strafeTo(collectFirstSet, new TranslationalVelConstraint(INTAKE_FORWARD_SPEED))
                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        telemetry.addData("Runtime", runtime.seconds());
        Actions.runBlocking(action);
    }
}