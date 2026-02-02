package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

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
    private static final double FLYWHEEL_RPM_FAR = 3400.0;
    private static final double FLYWHEEL_RPM_MID = 2800.0;
    private static final double FLYWHEEL_RPM_CLOSE = 2200.0;

    private final double ROTATION_MIN_POS = -1500;
    private final double ROTATION_MAX_POS = 2100;

    // PID Coefficients
    public static double rotation_kP = 0.02, rotation_kI = 0.0, rotation_kD = 0.0, rotation_kF = 0.003;
    public static double flywheel_kP = 0.001, flywheel_kI = 0.0, flywheel_kD = 0.0, flywheel_kF = 0.0002;

    // State
    private double targetAngle = 0;
    private double targetRPM = 0;
    public double rotationOutput = 0;
    private double flywheelOutput = 0;

    // PID Internal State
    private double rotation_lastError = 0, rotation_lastTime = 0, rotation_integral = 0;
    private double flywheel_lastError = 0, flywheel_lastTime = 0, flywheel_integral = 0;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        turret = hardwareMap.get(DcMotorEx.class, "turretTurn");

        // Flywheel Setup
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Used as encoder source

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
        if (dist < 55) return FLYWHEEL_RPM_CLOSE;
        if (dist < 100) return FLYWHEEL_RPM_MID;
        return FLYWHEEL_RPM_FAR;
    }

    public double autoPitch(double dist) {
        if (dist < 55) return 0.3;
        if (dist < 130) return 0.000862069 * dist + 0.637414;
        return 0.77; // Far pitch
    }

    // --- PID Updates ---

    public void updateRotationPID(double currentTimeSeconds, double tx, double rotationVelocity) {
        double currentAngle = getRotationPosition();
        double error = tx;
        double dt = Math.max(currentTimeSeconds - rotation_lastTime, 0.001); // Avoid div/0

        double derivative = (error - rotation_lastError) / dt;
        rotation_integral += error * dt;

        double feedforward = rotationVelocity * rotation_kF;
        double output = (rotation_kP * error) + (rotation_kI * rotation_integral) + (rotation_kD * derivative) + feedforward;
        rotationOutput = Math.max(-1.0, Math.min(1.0, output));
        if (rotationOutput > 0 && getRotationPosition() > ROTATION_MAX_POS) {
            rotationOutput = 0;
        }
        if (rotationOutput < 0 && getRotationPosition() < ROTATION_MIN_POS) {
            rotationOutput = 0;
        }
        rotation_lastError = error;
        rotation_lastTime = currentTimeSeconds;
    }

    public void updateFlywheelPID(double currentTimeSeconds) {
        double currentRPM = getFlywheelRPM();
        double error = targetRPM - currentRPM;
        double dt = Math.max(currentTimeSeconds - flywheel_lastTime, 0.001);

        double derivative = (error - flywheel_lastError) / dt;
        flywheel_integral += error * dt;

        double feedforward = targetRPM * flywheel_kF;
        double output = (flywheel_kP * error) + (flywheel_kI * flywheel_integral) + (flywheel_kD * derivative) + feedforward;

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

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
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

    public double getFlywheelRPM() {
        return (flywheel.getVelocity() * 60.0) / ENCODER_TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTargetAngle() {
        return targetAngle;
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

}