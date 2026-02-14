package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.PIDConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    // Hardware
    public DcMotorEx flywheel;//the one with encoder
    public DcMotorEx flywheel2;
    public DcMotorEx turret;

    public Servo pitch;

    // Constants
    private static final double ENCODER_TICKS_PER_REV = 28.0;
    private static final double DEGREES_PER_TICK = 90.0 / 5850.0;

    // Tuning
    private static final double FLYWHEEL_RPM_FAR = 3150.0;
    private static final double FLYWHEEL_RPM_MID = 2800.0;
    private static final double FLYWHEEL_RPM_CLOSE = 2200.0;

    private final double ROTATION_MIN_POS = -1120;
    private final double ROTATION_MAX_POS = 950;

    // State
    private double targetRPM = 0;
    public double rotationOutput = 0;
    private double flywheelOutput = 0;

    // PID Internal State
    private double limelightRotation_lastError = 0, limelightRotation_lastTime = 0, limelightRotation_integral = 0;
    private double encoderRotation_lastError = 0, encoderRotation_lastTime = 0, encoderRotation_integral = 0;

    private double flywheel_lastError = 0, flywheel_lastTime = 0, flywheel_integral = 0;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turret = hardwareMap.get(DcMotorEx.class, "turretTurn");

        // Flywheel Setup
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Setup
        pitch = hardwareMap.get(Servo.class, "pitch");

        pitch.setDirection(Servo.Direction.FORWARD);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // --- Auto Calculations ---

    public double autoRPM(double dist) {
        if (dist < 30) return FLYWHEEL_RPM_CLOSE;
        if (dist < 225) return 4 * dist + 1970;
        return FLYWHEEL_RPM_FAR;
    }

    public double autoPitch(double dist) {
        if (dist < 30) return 0.3;
        if (dist < 225) return 0.655;
        return 0.55; // Far pitch
    }

    // --- PID Updates ---

    public void updateLimelightPID(double currentTimeSeconds, double tx, boolean isTeamRed) {
        if (limelightRotation_lastTime == 0) {
            limelightRotation_lastTime = currentTimeSeconds;
        }

        double error = (isTeamRed) ? tx + 0.5 : tx -0.5;
        double dt = Math.max(currentTimeSeconds - limelightRotation_lastTime, 0.001); // Avoid div/0

        double derivative = (error - limelightRotation_lastError) / dt;
        limelightRotation_integral += error * dt;

        double output = (LIMELIGHT_ROTATION_kP * error) + (LIMELIGHT_ROTATION_kI * limelightRotation_integral) + (LIMELIGHT_ROTATION_kD * derivative);
        if (Math.abs(error) > LIMELIGHT_FF_DEADZONE) {
            output += LIMELIGHT_ROTATION_kF * Math.signum(error);
        }
        rotationOutput = Math.max(-1.0, Math.min(1.0, output));
        if (rotationOutput > 0 && getRotationPosition() > ROTATION_MAX_POS) {
            rotationOutput = 0;
            limelightRotation_integral = 0;
        }
        if (rotationOutput < 0 && getRotationPosition() < ROTATION_MIN_POS) {
            rotationOutput = 0;
            limelightRotation_integral = 0;
        }
        limelightRotation_lastError = error;
        limelightRotation_lastTime = currentTimeSeconds;
    }

    public void updateEncoderPID(double currentTimeSeconds, double encoderTarget) {
        if (encoderRotation_lastTime == 0) {
            encoderRotation_lastTime = currentTimeSeconds;
        }

        double error = encoderTarget - getRotationPosition();
        double dt = Math.max(currentTimeSeconds - encoderRotation_lastTime, 0.001); // Avoid div/0

        double derivative = (error - encoderRotation_lastError) / dt;
        encoderRotation_integral += error * dt;

        double output = (ENCODER_ROTATION_kP * error) + (ENCODER_ROTATION_kI * encoderRotation_integral) + (ENCODER_ROTATION_kD * derivative);
        rotationOutput = -Math.max(-1.0, Math.min(1.0, output));
        if (rotationOutput > 0 && getRotationPosition() > ROTATION_MAX_POS) {
            rotationOutput = 0;
            encoderRotation_integral = 0;
        }
        if (rotationOutput < 0 && getRotationPosition() < ROTATION_MIN_POS) {
            rotationOutput = 0;
            encoderRotation_integral = 0;
        }
        encoderRotation_lastError = error;
        encoderRotation_lastTime = currentTimeSeconds;
    }


    public void updateFlywheelPID(double currentTimeSeconds) {
        double currentRPM = getFlywheelRPM();
        double error = targetRPM - currentRPM;
        double dt = Math.max(currentTimeSeconds - flywheel_lastTime, 0.001);

        double derivative = (error - flywheel_lastError) / dt;
        flywheel_integral += error * dt;

        double feedforward = targetRPM * FLYWHEEL_kF;
        double output = (FLYWHEEL_kP * error) + (FLYWHEEL_kI * flywheel_integral) + (FLYWHEEL_kD * derivative) + feedforward;

        if (error < 0.3) flywheelOutput = 0;
        flywheelOutput = Math.max(-1.0, Math.min(1.0, output));

        flywheel_lastError = error;
        flywheel_lastTime = currentTimeSeconds;
    }

    // --- Hardware Interaction ---
    public void applyRotationPower() {
        turret.setPower(rotationOutput);
    }

    public void overrideRotationPower(double power) {
        this.rotationOutput = power;
        applyRotationPower(); // Apply immediately for responsiveness
    }

    public void applyFlywheelPower() {
        if (Math.abs(targetRPM) < 1.0) {
            flywheel.setPower(0);
            flywheel2.setPower(0);
        } else {
            flywheel.setPower(flywheelOutput);
            flywheel2.setPower(flywheelOutput);
        }
    }

    // --- Helpers ---

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / ENCODER_TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void setPitch(double pos) {
        pitch.setPosition(pos);
    }

    public double getPitch() {
        return pitch.getPosition();
    }


    public double getRotationPosition() {
        return turret.getCurrentPosition();
    }


    public double angleToTarget(double xPos, double yPos, double heading, boolean isTeamRed) {
        double goalX = -69.0;
        double goalY = isTeamRed ? 64.0 : -64.0;
        double dx = goalX - xPos;
        double dy = goalY - yPos;
        return Math.toDegrees(Math.atan2(dy, dx)) - heading;
    }

    public double correctTurretAngle(double desiredAngle, double max, double min) {
        if (desiredAngle > max && desiredAngle - 360 > min) return desiredAngle - 360;
        if (desiredAngle < min && desiredAngle + 360 < max) return desiredAngle + 360;
        return Math.min(max, Math.max(desiredAngle, min));
    }

    public void resetLimelightPIDState() {
        limelightRotation_lastError = 0;
        limelightRotation_integral = 0;
        limelightRotation_lastTime = 0; // Reset timing to trigger initialization on next update
    }

    // Reset PID state for general direction PID controller
    public void resetEncoderPIDState() {
        encoderRotation_lastError = 0;
        encoderRotation_integral = 0;
        encoderRotation_lastTime = 0; // Reset timing to trigger initialization on next update
    }

    public double getDistance(double ta){
        return 176.6143*Math.pow(ta, -0.5101461);
    }
}