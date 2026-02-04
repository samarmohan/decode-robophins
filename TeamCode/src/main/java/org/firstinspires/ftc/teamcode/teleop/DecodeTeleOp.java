package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Decode TeleOp", group = "Competition")
public class DecodeTeleOp extends LinearOpMode {

    // Subsystems
    private final Turret turret = new Turret();
    private final Drivetrain drive = new Drivetrain();
    private final Limelight limelight = new Limelight();
    private final Intake intake = new Intake();
    private final Spindexer spindexer = new Spindexer();
    private final Tilt tilt = new Tilt();

    // Constants
    private static final double TURRET_MIN_DEG = -135.0;
    private static final double TURRET_MAX_DEG = 135.0;
    private static final double OVERRIDE_TRIGGER_THRESHOLD = 0.1;
    private static final double RPM_MINIMUM_FOR_SHOOTING = 2000.0;

    private static final boolean TESTING = false;

    // Enums for State Management
    private enum TurretMode {
        FULL_AUTO,
        IDLE,
        OVERRIDE
    }

    private double lastTime;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        turret.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.init(hardwareMap);
        spindexer.init(hardwareMap);
        tilt.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean isTeamRed = true;
        boolean shouldTilt = false;

        double pitchPosition = 1.0;
        double testingFlywheelTargetRPM = 0.0;
        double spindexerAngle = 0;
        double turretRotationTarget = 0.0;

        TurretMode turretMode = TurretMode.IDLE;
        ElapsedTime runtime = new ElapsedTime();

        limelight.start();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double currentTime = runtime.seconds();


            spindexer.update(
                    currentGamepad1.cross,          // In
                    currentGamepad1.square,         // Out
                    currentGamepad1.triangle,       // Off
                    currentGamepad1.right_trigger  // Shoot
            );
            
            drive.odometryUpdate();
            double xPos = drive.getPosX();
            double yPos = drive.getPosY();
            double heading = drive.getHeading();

            limelight.update(heading);

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                spindexer.resetTarget();
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                spindexer.shoot();
            }

            if (currentGamepad1.dpad_left && currentGamepad1.dpad_up) {
                isTeamRed = false;
            } else if (currentGamepad1.dpad_right && currentGamepad1.dpad_down) {
                isTeamRed = true;
            }
            if (currentGamepad1.touchpad){
                drive.setOdometryXY(64, isTeamRed ? -63.5 : 63.5);
                drive.setHeading(isTeamRed? -90: 90);
            }

            if (currentGamepad1.options) {
                drive.resetFieldCentric();
            }
                drive.driveFieldCentric(
                    -currentGamepad1.left_stick_y,
                    currentGamepad1.left_stick_x,
                    currentGamepad1.right_stick_x,
                    currentGamepad1.left_trigger
            );


            boolean isOverride = currentGamepad2.left_trigger > OVERRIDE_TRIGGER_THRESHOLD;
            boolean auto = true;

            // If in testing mode, bumpers should change target RPM instead
            if (!TESTING) {
                if (isOverride) {
                    turretMode = TurretMode.OVERRIDE;
                } else {
                    if (currentGamepad1.right_bumper) {
                        turretMode = TurretMode.FULL_AUTO;
                    }
                    else if (currentGamepad1.left_bumper) {
                        turretMode = TurretMode.IDLE;
                    }
                }
            } else {
                turretMode = TurretMode.FULL_AUTO;
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    testingFlywheelTargetRPM += 100;
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    testingFlywheelTargetRPM -= 100;
                }
            }

            double distance = Math.hypot((66.0) + xPos, (isTeamRed ? 66.0 : -66.0) - yPos);

            switch (turretMode) {
                case FULL_AUTO:
                    // Auto Aim
                    turret.setTargetAngle(turretRotationTarget);

                    // Auto Pitch
                    pitchPosition = turret.autoPitch(distance);

                    // Auto RPM
                    double setRpm = TESTING ? testingFlywheelTargetRPM : turret.autoRPM(distance);
                    turret.setTargetRPM(setRpm);
                    break;

                case IDLE:
                    // Auto Aim
                    turret.setTargetAngle(turretRotationTarget);

                    turret.setTargetRPM(0);
                    break;

                case OVERRIDE:
                    turret.setTargetRPM(turret.autoRPM(distance));
                    pitchPosition = turret.autoPitch(distance);

                    double rotationStick = -currentGamepad2.right_stick_x;
                    double manualPower = rotationStick * 0.5; // Half speed

                    if ((rotationStick > 0.1 && turret.getRotationPosition() < TURRET_MAX_DEG) ||
                            (rotationStick < -0.1 && turret.getRotationPosition() > TURRET_MIN_DEG)) {
                        turret.overrideRotationPower(manualPower);
                    } else {
                        turret.overrideRotationPower(0);
                    }

                    if (currentGamepad2.left_stick_y > 0.1 && pitchPosition <= 1.0) {
                        pitchPosition += 0.005;
                    } else if (currentGamepad2.left_stick_y < -0.1 && pitchPosition >= 0.0) {
                        pitchPosition -= 0.005;
                    }
                    break;
            }

            turret.setPitch(pitchPosition);

            if (turretMode != TurretMode.OVERRIDE) {
                if (limelight.isResultValid()) {
                    // Camera PID: Use precise aiming with AprilTag
                    turret.updateRotationPID(currentTime, limelight.getTx(), drive.getRotationVelocity());
                    turret.applyRotationPower();
                } else {
                    // General Direction PID: Compensate for robot rotation to maintain general goal orientation
                    turret.updateGeneralDirectionPID(currentTime, drive.getRotationVelocity());
                    turret.applyRotationPower();
                }
            }

            turret.updateFlywheelPID(currentTime);
            turret.applyFlywheelPower();

            boolean isFlywheelReady = turret.getFlywheelRPM() > RPM_MINIMUM_FOR_SHOOTING;

            intake.update(
                    currentGamepad1.cross,          // In
                    currentGamepad1.square,         // Out
                    currentGamepad1.triangle,       // Off
                    currentGamepad1.right_trigger,  // Shoot
                    isFlywheelReady
            );

            if (currentGamepad2.square && !previousGamepad2.square) {
                shouldTilt = !shouldTilt;
            }

            if (shouldTilt) {
                tilt.moveDown();
            } else {
                tilt.moveUp();
            }

            telemetry.addLine(String.valueOf(tilt.tilt.getPosition()));
            telemetry.addData("heading velocity ", drive.getRotationVelocity());
            telemetry.addData("Loop Speed (ms)", (currentTime-lastTime)*1000);
            lastTime = currentTime;
            telemetry.addData("Mode", turretMode);
            telemetry.addData("Testing?", TESTING);
            telemetry.addData("Team", isTeamRed ? "RED" : "BLUE");
            telemetry.addData("Pos", "X:%.1f Y:%.1f H:%.1f", xPos, yPos, heading);
            telemetry.addData("Distance", distance);
            telemetry.addLine("---------------------------------------");

            //color sensor testing
            /*
            telemetry.addData("Alpha: ", spindexer.getSensorAlphaSpin());
            telemetry.addData("Red: ", spindexer.getNormalizedRedSpin());
            telemetry.addData("Blue: ", spindexer.getNormalizedBlueSpin());
            telemetry.addData("Green:", spindexer.getNormalizedGreenSpin());
            telemetry.addData("Ball Detected:", spindexer.ballDetectedSpin());
            telemetry.addData("Green Detected:", spindexer.ballIsGreenSpin());
            telemetry.addData("Purple Detected:", spindexer.ballIsPurpleSpin());


            telemetry.addData("Alpha: ", spindexer.getSensorAlphaIntake());
            telemetry.addData("Red: ", spindexer.getNormalizedRedIntake());
            telemetry.addData("Blue: ", spindexer.getNormalizedBlueIntake());
            telemetry.addData("Green:", spindexer.getNormalizedGreenIntake());

             */
            telemetry.addData("intake Ball:", spindexer.intakeBall);
            telemetry.addData("spindexer Ball:", spindexer.spindexerBall);
            telemetry.addData("spindexer state", spindexer.state);
            telemetry.addData("spindexer is full?", spindexer.isFull());




            telemetry.addLine("---------------------------------------");

            //spindexer testing
            telemetry.addData("Current Balls", spindexer.getOrder());
            telemetry.addData("Spindexer Target", spindexer.getTargetAngle());
            telemetry.addData("Spindexer Pos", spindexer.getCurrentAngle());
            telemetry.addData("Spindexer Index Forward", spindexer.getSpindexerPosForward());
            telemetry.addLine("---------------------------------------");

            telemetry.addData("Limelight", "X:%.1f Y:%.1f", limelight.getTx(), limelight.getTa());
            telemetry.addData("Intake", intake.getState());

            telemetry.addLine("---------------------------------------");

            telemetry.addData("Target RPM", turret.getTargetRPM());
            telemetry.addData("Actual RPM", turret.getFlywheelRPM());
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Actual Angle", turret.getRotationPosition());
            telemetry.addData("Turret Pitch", turret.getPitch());
            telemetry.addData("Rotation Power", turret.rotationOutput);
            telemetry.update();
        }
        limelight.stop();
    }
}