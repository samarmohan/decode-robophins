package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp
public class OBJDecodeTeleOp extends LinearOpMode {
    Turret turret = new Turret();
    Drivetrain drive = new Drivetrain();

    Limelight limelight = new Limelight();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        turret.init(hardwareMap);
        drive.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double pitchPos = 1.0;
        boolean in = true;
        boolean spinningFront = false;
        boolean spinningUp = false;
        double targetVelocity = 0.0;
        boolean autoAim = false;
        double lastTA = 0.0;
        double dist = 0.0;
        boolean close = true;
        double rotationPos = 0.5;
        final double TURRET_ROTATION_GEAR_RATIO = 2.0;
        double turretAngle = (rotationPos-0.5)*450.0* TURRET_ROTATION_GEAR_RATIO;
        double llx = 0;
        double lly = 0;
        double llr = 0;
        final double mToIn = 39.37;
        boolean hasHeading = false;
        boolean limelightOn = true;
        final double TURRET_MIN_DEG = -90.0;
        final double TURRET_MAX_DEG = 180.0;
        double idealTurretRelDeg = 0;
        double limitedTurretDeg = 0;
        ElapsedTime runtime = new ElapsedTime();
        double turretOffset = (5.75/2); //inches
        limelight.start();

        waitForStart();
        runtime.reset();
        final boolean isTeamRed = true;


        ElapsedTime autoAimTimer = new ElapsedTime();
        telemetry.update();

        while(opModeIsActive()) {
            drive.odometryUpdate();
            double xPos = drive.getPosX();
            double yPos = drive.getPosY();
            double heading = drive.getHeading();

            double standardizedHeading = turret.standardizeAngle(heading);
            double distanceL = Math.hypot(144 - xPos, 144 - yPos);
            double distanceR = Math.hypot(xPos, 144 - yPos);
            
            telemetry.addData("X (in)", xPos);
            telemetry.addData("Y (in)", yPos);
            telemetry.addData("Heading (deg)", heading);

            //telemetry.addData("Distance from Top Left (in)",F distanceL);
            //telemetry.addData("Distance from Top Right (in)", distanceR);
            
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            
            double y = (-currentGamepad1.left_stick_y);
            double x = currentGamepad1.left_stick_x;
            double rx = currentGamepad1.right_stick_x;
            double accelerator = gamepad1.left_trigger;

            if (gamepad1.options) {
                drive.resetFieldCentric();
            }

            drive.drive(y, x, rx, accelerator);
            //need to redo distance calculations
            if (currentGamepad1.left_bumper) {
                dist = turret.taToDistFar(lastTA);
                targetVelocity = turret.farRPM(dist);
                turret.setFlywheelRPM(targetVelocity);
                close = false;
            }

            if (currentGamepad1.b) {
                targetVelocity = 0.0;
                autoAim=false;
                turret.flywheelStop();
            }

            if (currentGamepad1.right_bumper) {
                dist = turret.taToDistClose(lastTA);
                pitchPos = 1-turret.closePitch(dist);
                targetVelocity = turret.closeRPM(dist);
                turret.setFlywheelRPM(targetVelocity);
                close = true;
            }


        if (autoAim) {
            idealTurretRelDeg = turret.angleToTarget(xPos, yPos, heading, isTeamRed);
        }

            turret.setPitch(pitchPos);
            //right trigger shoots

            //x toggle off/in
            //y toggles in/out
            

            //limelight
            limelight.update(heading, limelightOn);
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                limelightOn = !limelightOn;
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                autoAim = !autoAim;
            }
            if (currentGamepad1.a && currentGamepad1.b) {
                limelight.setFieldHeading();
            }
            if (autoAim) {
                rotationPos = turret.posFromAngle(turret.correctTurretAngle(idealTurretRelDeg, TURRET_MAX_DEG, TURRET_MIN_DEG));
            }

            turret.rightR.setPosition(rotationPos);
            turret.leftR.setPosition(rotationPos);


            if (currentGamepad1.dpad_down && pitchPos <= 1.0) {
                pitchPos += 0.005;
            } else if (currentGamepad1.dpad_up && pitchPos >= 0.2D) {
                pitchPos -= 0.005;
            }
            
            double pitchStick = (currentGamepad2.right_stick_y);
            if (pitchStick > 0.1D && pitchPos >= 0.0) {
                pitchPos += 0.005;
            } else if (pitchStick < -0.1D && pitchPos <= 1.0) {
                pitchPos -= 0.005;
            }
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Turret Pitch", turret.pitch.getPosition());
            telemetry.addData("Stick Pitch", pitchStick);
            telemetry.addData("Turret Rotation Position", rotationPos);
            telemetry.addData("Raw turret target: ", idealTurretRelDeg);
            telemetry.addData("turret Target angle: ", limitedTurretDeg);
            telemetry.addData("Flywheel Velocity", turret.getFlywheelVelocity());
            telemetry.addData("Target RPM", targetVelocity);
            telemetry.addData("Auto Aim?", autoAim);
            telemetry.addData("RightR Pos", turret.rightR.getPosition());
            telemetry.addData("LeftR Pos", turret.leftR.getPosition());
            telemetry.addData("limelight x", limelight.getLlx());
            telemetry.addData("limelight y", limelight.getLly());
            telemetry.addData("limelight rot", limelight.getLlh());
            telemetry.update();
        }

    }
}

