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
        telemetry.addData("Status", "DEAD");
        telemetry.update();

        turret.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.init(hardwareMap);
        spindexer.init(hardwareMap, intake);
        tilt.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean isTeamRed = true;
        boolean shouldTilt = false;

        double pitchPosition = 1.0;
        double testingFlywheelTargetRPM = 0.0;
        boolean previousAprilTagVisible = false;

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

            boolean isFlywheelReady = turret.getFlywheelRPM() > RPM_MINIMUM_FOR_SHOOTING;


            spindexer.update(
                    currentGamepad1.cross,          // In
                    currentGamepad1.right_trigger,  // Shoot
                    isFlywheelReady
            );

            drive.autoLight(spindexer.isFull(),!spindexer.hasBalls());

            drive.odometryUpdate();
            double xPos = drive.getPosX();
            double yPos = drive.getPosY();
            double heading = drive.getHeading();

            limelight.update(heading);

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                spindexer.setTargetAngle(spindexer.getCurrentAngle()+60);
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                spindexer.setTargetAngle(spindexer.getCurrentAngle());
            }

            if (currentGamepad2.dpad_left && currentGamepad2.dpad_up) {
                isTeamRed = false;
                limelight.setPipeline(0);
            } else if (currentGamepad2.dpad_right && currentGamepad2.dpad_down) {
                isTeamRed = true;
                limelight.setPipeline(1);
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

            double distance = turret.getDistance(limelight.getTa());

            switch (turretMode) {
                case FULL_AUTO:
                    // Auto Pitch
                    pitchPosition = turret.autoPitch(distance);

                    // Auto RPM
                    double setRpm = TESTING ? testingFlywheelTargetRPM : turret.autoRPM(distance);
                    turret.setTargetRPM(setRpm);
                    break;

                case IDLE:
                    turret.setTargetRPM(0);
                    break;

                case OVERRIDE:
                    //turret.setTargetRPM(turret.autoRPM(distance));
                    //pitchPosition = turret.autoPitch(distance);

                    /*
                    double rotationStick = -currentGamepad2.right_stick_x;
                    double manualPower = rotationStick * 0.5; // Half speed

                    if ((rotationStick > 0.1 && turret.getRotationPosition() < TURRET_MAX_DEG) ||
                            (rotationStick < -0.1 && turret.getRotationPosition() > TURRET_MIN_DEG)) {
                        turret.overrideRotationPower(manualPower);
                    } else {
                        turret.overrideRotationPower(0);
                    }


                     */
                    if (currentGamepad2.left_stick_y > 0.1 && pitchPosition <= 1.0) {
                        pitchPosition += 0.005;
                    } else if (currentGamepad2.left_stick_y < -0.1 && pitchPosition >= 0.0) {
                        pitchPosition -= 0.005;
                    }
                    break;
            }

            turret.setPitch(pitchPosition);

            if (turretMode != TurretMode.OVERRIDE) {
                boolean currentAprilTagVisible = limelight.isResultValid();

                /*
                // Detect mode switching and reset PID states accordingly
                if (currentAprilTagVisible != previousAprilTagVisible) {
                    if (currentAprilTagVisible) {
                        // Switching from general direction to camera PID
                        turret.resetLimelightPIDState();
                    } else {
                        // Switching from camera to general direction PID
                        turret.resetEncoderPIDState();
                    }
                }

                previousAprilTagVisible = currentAprilTagVisible;

                 */

                if (currentAprilTagVisible) {
                    turret.updateLimelightPID(currentTime, limelight.getTx(), isTeamRed);
                    turret.applyRotationPower();
                } else {
                    turret.updateEncoderPID(currentTime, 0);
                    turret.applyRotationPower();
                }
            }

            turret.updateFlywheelPID(currentTime);
            turret.applyFlywheelPower();

            if (currentGamepad2.square && !previousGamepad2.square) {
                shouldTilt = !shouldTilt;
            }

            if (shouldTilt) {
                tilt.moveDown();
            } else {
                tilt.moveUp();
            }

            telemetry.addData("Loop Speed (ms)", (currentTime-lastTime)*1000);
            lastTime = currentTime;
            telemetry.addData("Mode", turretMode);
            telemetry.addData("Testing?", TESTING);
            telemetry.addData("Team", isTeamRed ? "RED" : "BLUE");
            telemetry.addData("Pos", "X:%.1f Y:%.1f H:%.1f", xPos, yPos, heading);

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

            telemetry.addLine("---------------------------------------");
            telemetry.addData("Target RPM", turret.getTargetRPM());
            telemetry.addData("Actual RPM", turret.getFlywheelRPM());
            telemetry.addLine("---------------------------------------");
            telemetry.addData("Turret Pitch", turret.getPitch());
            telemetry.addLine("---------------------------------------");

            telemetry.addData("Actual Angle", turret.getRotationPosition());
            telemetry.addData("Limelight Valid?", limelight.isResultValid());
            telemetry.addData("Limelight Position", limelight.getTx());
            telemetry.addData("limelight distance", turret.getDistance(limelight.getTa()));
            telemetry.addData("Rotation Power", turret.rotationOutput);

            telemetry.addLine("---------------------------------------");
            telemetry.addData("Spindexer State", spindexer.getState());
            telemetry.addData("Intake State", intake.getState());
            telemetry.addData("Ball in Spindexer", spindexer.getSpindexerBall());
            telemetry.addData("Ball Count", spindexer.isFull() ? "3 (FULL)" : spindexer.hasBalls() ? "1-2" : "0");
            telemetry.addData("Current Order", spindexer.getOrder());
            telemetry.addData("Spindexer Target", spindexer.target);
            telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
            telemetry.addData("Has Entered Intake", spindexer.hasEnteredIntaking);
            telemetry.addData("Has Entered Ready To Shoot", spindexer.hasEnteredReadyToShoot);
            telemetry.update();
        }
        limelight.stop();
    }
}