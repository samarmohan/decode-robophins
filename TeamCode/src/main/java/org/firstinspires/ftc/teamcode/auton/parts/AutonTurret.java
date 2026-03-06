package org.firstinspires.ftc.teamcode.auton.parts;


import static org.firstinspires.ftc.teamcode.PIDConstants.*;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonTurret {
    public Limelight3A limelight;

    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;

    public DcMotorEx turret;
    public Servo pitch;

    private static final double ENCODER_TICKS_PER_REV = 28.0;
    private static final double SECONDS_PER_MINUTE = 60.0;

    private final double ROTATION_MIN_POS = -1120;
    private final double ROTATION_MAX_POS = 950;

    private double rotationOutput = 0;

    public String team;

    public static final int AIMING_PIPELINE_BLUE = 0;
    public static final int AIMING_PIPELINE_RED = 0;

    public double lastTx;


    public AutonTurret(HardwareMap hardwareMap, Limelight3A limelight, String team) {
        this.limelight = limelight;
        this.team = team;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turret = hardwareMap.get(DcMotorEx.class, "turretTurn");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setVelocityPIDFCoefficients(300, 10, 1, 1.0);
        flywheel2.setVelocityPIDFCoefficients(300, 10, 1, 1.0);

        pitch = hardwareMap.get(Servo.class, "pitch");
        pitch.setDirection(Servo.Direction.FORWARD);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public Action setFlywheelRPM(double rpm) {
        return packet -> {
            packet.put("Flywheel Target RPM", rpm);
            flywheel.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
            flywheel2.setVelocity(rpm * (ENCODER_TICKS_PER_REV / SECONDS_PER_MINUTE));
            packet.put("Flywheel RPM", (flywheel.getVelocity() * 60.0) / ENCODER_TICKS_PER_REV);
            return true;
        };
    }

    public Action rotateRight(int angle) {
        return packet -> {
            double pos = -turret.getCurrentPosition();
            if (pos < angle)  {
                turret.setPower(0.4);
                return true;
            } else {
                turret.setPower(0);
                return false;
            }
        };
    }

    public Action rotateLeft(int angle) {
        return packet -> {
            double pos = -turret.getCurrentPosition();
            if (pos > angle)  {
                turret.setPower(-0.4);
                return true;
            } else {
                turret.setPower(0);
                return false;
            }
        };
    }

    public Action switchToAimingPipeline() {
        return packet -> {
            int pipeline = team.equals("RED") ? AIMING_PIPELINE_RED :  AIMING_PIPELINE_BLUE;
            limelight.pipelineSwitch(pipeline);
            packet.put("limelight", pipeline);
            return false;
        };
    }

    public Action updateLimelightPID() {
        return packet -> {
            double rotationPos = -turret.getCurrentPosition();
            double tx;
            boolean isTeamRed = team.equals("RED");

            boolean currentAprilTagVisible = limelight.getLatestResult().isValid();
            if (currentAprilTagVisible) {
                tx = limelight.getLatestResult().getTx();
            } else {
                tx = lastTx;
            }
            lastTx = tx;


            double error = (isTeamRed) ? tx + 1 : tx -1;

            double output = (AUTON_LIMELIGHT_ROTATION_kP * error);
            if (Math.abs(error) > AUTON_LIMELIGHT_FF_DEADZONE) {
                output += AUTON_LIMELIGHT_ROTATION_kF * Math.signum(error);
            }
            rotationOutput = Math.max(-1.0, Math.min(1.0, output));
            if (rotationOutput > 0 && rotationPos > ROTATION_MAX_POS) {
                rotationOutput = 0;
            }
            if (rotationOutput < 0 && rotationPos < ROTATION_MIN_POS) {
                rotationOutput = 0;
            }
            packet.put("Rotation Output", rotationPos);
            packet.put("Tx", tx);
            packet.put("Rotation Pos", rotationPos);
            turret.setPower(rotationOutput);
            return true;
        };
    }


    public Action setPitchPosition(double pitchPos) {
            return packet -> {
                pitch.setPosition(pitchPos);
                packet.put("Pitch Position", pitchPos);
                return false;
            };
        }
}