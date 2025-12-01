package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auton.parts.Intake;


@Autonomous(name = "BlueCloseAuton")
public class BlueCloseAuton extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intake intake = new Intake(hardwareMap);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                intake.intakeHold(),
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(0, -10))
                        .strafeTo(new Vector2d(-20, -20))
                        .strafeTo(new Vector2d(-20, 0))
                        .afterTime(1, intake.intakeOff())
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