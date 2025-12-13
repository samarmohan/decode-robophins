package org.firstinspires.ftc.teamcode.teleop;


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

        final double TURRET_MIN_DEG = -90.0;
        final double TURRET_MAX_DEG = 90.0;
        final double TURRET_OFFSET = (5.75/2); //inches

        double pitchPos = 1.0;
        double rotationAngle = 0;

        double idealTurretRelDeg = 0;
        double limitedTurretDeg = 0;
        
        boolean autoAim = false;
        boolean override = false;
        boolean autoPower = false;
    
        ElapsedTime runtime = new ElapsedTime();
    
        limelight.start();
        waitForStart();
        runtime.reset();
        boolean isTeamRed = true;


        ElapsedTime autoAimTimer = new ElapsedTime();
        telemetry.update();

        while(opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            
            drive.odometryUpdate();
            double xPos = drive.getPosX();
            double yPos = drive.getPosY();
            double heading = drive.getHeading();
            
            double currentTime = runtime.seconds();
            
            boolean button_a = currentGamepad1.a && !currentGamepad1.b;
            boolean button_b = currentGamepad1.b && !currentGamepad1.a;
            boolean button_x = currentGamepad1.x && !currentGamepad1.y;
            boolean button_y = currentGamepad1.y && !currentGamepad1.x;
            boolean toggle_x = button_x && !previousGamepad1.x;
            boolean toggle_y = button_y && !previousGamepad1.y;

            double distanceR = Math.hypot(66 + xPos, 66 - yPos);
            double distanceB = Math.hypot(66 + xPos, 66 + yPos);
            
            double distance;
            if (isTeamRed){
                distance = distanceR;
            } else{
                distance = distanceB;
            }
            
            telemetry.addData("X (in)", xPos);
            telemetry.addData("Y (in)", yPos);
            telemetry.addData("Heading (deg)", heading);
            
            
            double y = (-currentGamepad1.left_stick_y);
            double x = currentGamepad1.left_stick_x;
            double rx = currentGamepad1.right_stick_x;
            double accelerator = gamepad1.left_trigger;

            //resets field centric heading
            if (gamepad1.options) {
                drive.resetFieldCentric();
            }

            //runs drive code
            drive.driveFieldCentric(y, x, rx, accelerator);


            if (currentGamepad1.right_bumper) {
                //autoPower = true;
                //autoAim = true;
                turret.setTargetRPM(3000);
            }
            if (currentGamepad1.left_bumper) {
                //autoPower = false;
                //autoAim = false;
                turret.setTargetRPM(0); // Set Target
            }
            if (autoPower){
                pitchPos = turret.autoPower(distance);
            }
            
            turret.setPitch(pitchPos);
            
            if (autoAim) {
                idealTurretRelDeg = turret.angleToTarget(xPos, yPos, heading, isTeamRed);
                rotationAngle = turret.correctTurretAngle(idealTurretRelDeg, TURRET_MAX_DEG, TURRET_MIN_DEG);
                turret.setTargetAngle(rotationAngle);
            }
            //run intake state machine
            intake.loop(toggle_y, toggle_x, currentGamepad1.right_trigger);
            //right trigger shoots

            //x toggle off/in

            //y toggles in/out


            //limelight
            limelight.update(heading);
            double llX = limelight.getMT2X();
            double llY = limelight.getMT2Y();
            double llH = limelight.getLlh();
            
            if (currentGamepad1.dpad_left && currentGamepad1.dpad_up) {
                drive.setOdometryXY(llX, llY);
                drive.setHeading(llH);
                isTeamRed = false;
            } else if (currentGamepad1.dpad_right && currentGamepad1.dpad_down) {
                drive.setOdometryXY(llX, llY);
                drive.setHeading(llH);
                isTeamRed = true;
            }
            
            if (currentGamepad2.left_trigger > 0.1) override = true;
            else override = false;

            if (override) {
                autoAim = false;
                autoPower = false;

                // Manual Rotation
                double rotationStick = -(currentGamepad2.right_stick_x);
                double manualPower = rotationStick / 2;

                // We inject the manual power directly into the output variable
                // This bypasses the PID calculation loop's authority for this frame
                if ((rotationStick > 0.1 && rotationAngle < TURRET_MAX_DEG) ||
                        (rotationStick < -0.1 && rotationAngle > TURRET_MIN_DEG)) {
                    turret.overrideRotationPower(manualPower);
                } else {
                    turret.overrideRotationPower(0);
                }

                // Manual Pitch
                if (currentGamepad2.left_stick_y > 0.1 && pitchPos >= 0.0) pitchPos += 0.005;
                else if (currentGamepad2.left_stick_y < -0.1 && pitchPos <= 1.0) pitchPos -= 0.005;
            } else {
                // If NOT overriding, we calculate PID
                turret.updateRotationPID(currentTime);
            }

            // --- Finalize & Apply ---
            // Flywheel always runs PID (autoPower sets target, manual sets 0)
            turret.updateFlywheelPID(currentTime);

            // Actually write to motors
            turret.applyRotationPower();
            turret.applyFlywheelPower();
            
            telemetry.addData("Run Time", runtime.toString());
            /*
            telemetry.addData("Is the team Red", isTeamRed);
            telemetry.addData("Distance From Goal", distance);
            telemetry.addData("Turret Pitch", turret.pitch.getPosition());
            
            
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Real Angle", turret.getRotationPosition());
            telemetry.addData("Rotation Power", turret.getRotationOutput());
            */
            
            telemetry.addData("Target RPM", turret.getTargetRPM());
            telemetry.addData("Real RPM", turret.getFlywheelVelocityRPM());
            telemetry.addData("Flywheel Power", turret.getFlywheelOutput());
            /*
            telemetry.addData("Auto Aim?", autoAim);
            telemetry.addData("Auto Power?", autoPower);
            telemetry.addData("Override?", override);
            telemetry.addData("rotation pid power", turret.getRotationOutput());
            telemetry.addData("Limelight Rotation", limelight.getLlh());
            telemetry.addData("Limelight X", limelight.getMT2X());
            telemetry.addData("Limelight Y", limelight.getMT2Y());
            */
            telemetry.update();
        }

    }
}

