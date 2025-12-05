package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DecodeTeleOp extends LinearOpMode {
    Turret turret = new Turret();
    Drivetrain drive = new Drivetrain();
    Limelight limelight = new Limelight();
    Intake intake = new Intake();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        turret.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.init(hardwareMap);


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        double pitchPos = 1.0;
        double targetVelocity = 0.0;
        boolean autoAim = false;
        double rotationPos = 0.5;

        boolean limelightOn = true;
        final double TURRET_MIN_DEG = -90.0;
        final double TURRET_MAX_DEG = 180.0;
        double idealTurretRelDeg = 0;
        double limitedTurretDeg = 0;
        ElapsedTime runtime = new ElapsedTime();
        final double TURRET_OFFSET = (5.75/2); //inches
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

            //resets field centric heading
            if (gamepad1.options) {
                drive.resetFieldCentric();
            }

            //runs drive code
            drive.drive(y, x, rx, accelerator);


            //need to redo distance calculations

            if (currentGamepad1.left_bumper) {
                /*
                dist = turret.taToDistFar(lastTA);
                targetVelocity = turret.farRPM(dist);
                turret.setFlywheelRPM(targetVelocity);
                 */
                turret.setFlywheelRPM(3400);

            }

            if (currentGamepad1.b) {
                targetVelocity = 0.0;
                autoAim=false;
                turret.flywheelStop();
            }

            if (currentGamepad1.right_bumper) {
                /*
                dist = turret.taToDistClose(lastTA);
                pitchPos = 1-turret.closePitch(dist);
                targetVelocity = turret.closeRPM(dist);
                turret.setFlywheelRPM(targetVelocity);
                close = true;
                */
                turret.setFlywheelRPM(2800);
            }


            if (autoAim) {
                idealTurretRelDeg = turret.angleToTarget(xPos, yPos, heading, isTeamRed);
            }

            turret.setPitch(pitchPos);

            //run intake state machine
            intake.loop(currentGamepad1.y, currentGamepad1.x, currentGamepad1.right_trigger);
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

            //turret.setRotation(rotationPos);


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
            telemetry.addData("Turret Pos", turret.getRotationRealPosition());
            telemetry.addData("limelight x", limelight.getLlx());
            telemetry.addData("limelight y", limelight.getLly());
            telemetry.addData("limelight rot", limelight.getLlh());
            telemetry.update();
        }

    }
}

