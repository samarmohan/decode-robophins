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
        turret.limelight.start();

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
            double desRot = 0.0;
            double desTurretAngleDeg = 0.0;

        if (autoAim) {
            idealTurretRelDeg = turret.angleToTarget(xPos, yPos, heading, isTeamRed);
}

            turret.setPitch(pitchPos);
            if (currentGamepad1.right_trigger > 0.2) {
                if (turret.getFlywheelVelocity() > 2000.0) {
                    spinningUp = true;
                    spinningFront = true;
                    in = true;
                    turret.frontIntake.setPower(1.0);
                    turret.backIntake.setPower(1.0);
                } else {
                spinningUp = false;
                turret.backIntake.setPower(-0.5);
                    
                }
            }else{
                turret.backIntake.setPower(-0.5);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                if (spinningFront) {
                    spinningFront = false;
                    turret.frontIntake.setPower(0.0);
                    turret.backIntake.setPower(0.0);
                } else {
                    spinningFront = true;
                    turret.frontIntake.setPower(1.0);
                    turret.backIntake.setPower(-0.5);
                }
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                in=!in;}
            if (spinningFront && in){
                turret.frontIntake.setPower(1.0);
            }
            else if (spinningFront&&!in){
                turret.frontIntake.setPower(-1.0);
            }
        
            
            turret.limelight.updateRobotOrientation(standardizedHeading);
            LLResult result = turret.limelight.getLatestResult();
            
            if (result != null){
                if (result.isValid()){
                    Pose3D botpose = result.getBotpose();
                    llx = mToIn * botpose.getPosition().x;
                    lly = mToIn * botpose.getPosition().y;
                    llr = botpose.getOrientation().getYaw();
                
                    if (currentGamepad1.back){
                        //drive.odometry.setHeading(llr, AngleUnit.DEGREES);
                    }
                    if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                        limelightOn = !limelightOn;
                    }
                    if (limelightOn){
                        if (!hasHeading){
                            hasHeading = true;
                            //drive.odometry.setHeading(llr, AngleUnit.DEGREES);
                        }else{
                            Pose3D pose_mt2 = result.getBotpose_MT2();
                            double mt2x = mToIn * pose_mt2.getPosition().x;
                            double mt2y = mToIn * pose_mt2.getPosition().y;
                            //drive.odometry.setPosX(mt2y + 72, DistanceUnit.INCH);
                            //drive.odometry.setPosY((-mt2x) + 72, DistanceUnit.INCH);
                        }
                    }
                }
            }
            if (currentGamepad1.a && !previousGamepad1.a) {
                autoAim = !autoAim;
            }
            if (autoAim) {
                limitedTurretDeg = turret.correctTurretAngle(idealTurretRelDeg, TURRET_MAX_DEG, TURRET_MIN_DEG);
                rotationPos = limitedTurretDeg / 820.0 + 0.5;
            }

            rotationPos = Math.max(0.0, Math.min(1.0, rotationPos));

            turret.rightR.setPosition(rotationPos);
            turret.leftR.setPosition(rotationPos);

            double rotationStick = currentGamepad2.left_stick_x;
            if (rotationStick > 0.1 && rotationPos <= 500.0) {
                //turret.rotation.setPower(-0.6D * rotationStick);
            } else if (rotationStick < -0.1 && rotationPos >= -3000.0) {
                //turret.rotation.setPower(-0.6 * rotationStick);
            }

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
            telemetry.addData("Desired Angle Pos",desRot);
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
            telemetry.addData("limelight x", llx);
            telemetry.addData("limelight y", lly);
            telemetry.addData("limelight rot", llr);
            telemetry.update();
        }

    }
}

