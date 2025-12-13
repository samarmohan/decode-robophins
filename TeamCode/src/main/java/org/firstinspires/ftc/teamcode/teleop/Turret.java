package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Turret {
    // --- Hardware ---
    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;
    public CRServo rightR;
    public CRServo leftR;
    public Servo pitch;

    // --- Constants ---
    private static final double ENCODER_TICKS_PER_REV = 28.0;

    private static final double DEGREES_PER_TICK = 90.0 / 5850.0;
    private static final double FLYWHEEL_RPM_FAR = 3400.0;
    private static final double FLYWHEEL_RPM_MID = 2550.0;
    private static final double FLYWHEEL_RPM_CLOSE = 2200.0;

    // =========================================================
    //               PIDF COEFFICIENTS
    // =========================================================

    // Rotation PID
    public double rotation_kP = 0.015;
    public double rotation_kI = 0.0;
    public double rotation_kD = 0.0;
    public double rotation_kF = 0.0;

    // Flywheel PIDF
    public double flywheel_kP = 0.0001;
    public double flywheel_kI = 0;
    public double flywheel_kD = 0;
    public double flywheel_kF = 0.0002; // Feedforward (approx 1/MaxRPM)

    // =========================================================
    //               STATE VARIABLES
    // =========================================================

    // Targets
    private double targetAngle = 0;
    private double targetRPM = 0;

    // Outputs (Motor Power -1.0 to 1.0)
    private double rotationOutput = 0;
    private double flywheelOutput = 0;

    // PID State Tracking
    private double rotation_lastError = 0;
    private double rotation_lastTime = 0;
    private double rotation_integral = 0;

    private double flywheel_lastError = 0;
    private double flywheel_lastTime = 0;
    private double flywheel_integral = 0;

    // =========================================================
    //               INIT
    // =========================================================

    public void init(HardwareMap hardwareMap) {
        // Flywheel Hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret Hardware
        pitch = hardwareMap.get(Servo.class, "pitch");
        rightR = hardwareMap.get(CRServo.class, "rightR");
        leftR  = hardwareMap.get(CRServo.class, "leftR");

        pitch.setDirection(Servo.Direction.REVERSE);
        rightR.setDirection(CRServo.Direction.FORWARD);
        leftR.setDirection(CRServo.Direction.FORWARD);
    }

    // =========================================================
    //               SETTERS (Targets)
    // =========================================================

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    // =========================================================
    //               GETTERS (Sensors & Status)
    // =========================================================

    public double getRotationPosition() {
        // Converted to degrees
        return flywheel2.getCurrentPosition() * DEGREES_PER_TICK;
    }

    public double getFlywheelVelocityRPM() {
        // Converted to RPM
        return (flywheel.getVelocity() * 60.0) / ENCODER_TICKS_PER_REV;
    }

    public double getRotationOutput() {
        return rotationOutput;
    }

    public double getFlywheelOutput() {
        return flywheelOutput;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    // =========================================================
    //               PID UPDATE LOOPS
    // =========================================================

    /**
     * Calculates Rotation PID. Does NOT apply power.
     * @return calculated motor power (-1.0 to 1.0)
     */
    public double updateRotationPID(double currentTimeSeconds) {
        double currentAngle = getRotationPosition();
        double error = targetAngle - currentAngle;
        double dt = currentTimeSeconds - rotation_lastTime;

        if (dt <= 0.0001) dt = 0.001;

        double derivative = (error - rotation_lastError) / dt;
        rotation_integral += error * dt;

        // Calculate
        double output = (rotation_kP * error) + (rotation_kI * rotation_integral) + (rotation_kD * derivative);

        // Clamp
        rotationOutput = Math.max(-1.0, Math.min(1.0, output));

        // Save State
        rotation_lastError = error;
        rotation_lastTime = currentTimeSeconds;

        return rotationOutput;
    }

    /**
     * Calculates Flywheel PID. Does NOT apply power.
     * @return calculated motor power (-1.0 to 1.0)
     */
    public double updateFlywheelPID(double currentTimeSeconds) {
        double currentRPM = getFlywheelVelocityRPM();
        double error = targetRPM - currentRPM;
        double dt = currentTimeSeconds - flywheel_lastTime;

        if (dt <= 0.0001) dt = 0.001;

        double derivative = (error - flywheel_lastError) / dt;
        flywheel_integral += error * dt;

        double feedforward = targetRPM * flywheel_kF;

        // Calculate
        double output = (flywheel_kP * error) + (flywheel_kI * flywheel_integral) + (flywheel_kD * derivative) + feedforward;

        // Clamp
        flywheelOutput = Math.max(-1.0, Math.min(1.0, output));

        // Save State
        flywheel_lastError = error;
        flywheel_lastTime = currentTimeSeconds;

        return flywheelOutput;
    }

    // =========================================================
    //               APPLY POWER (Hardware Write)
    // =========================================================

    public void applyRotationPower() {
        // Add logic here if you want to disable rotation when power is negligible
        rightR.setPower(rotationOutput);
        // leftR.setPower(rotationOutput);
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

    // Allow manual override of the output variable if needed (e.g. driver control)
    public void overrideRotationPower(double power) {
        this.rotationOutput = power;
    }

    // =========================================================
    //               HELPER LOGIC
    // =========================================================

    public double autoPower(double dist){
        if (dist < 55){
            setTargetRPM(FLYWHEEL_RPM_CLOSE);
            return closePitch(dist);
        }
        if (dist < 100){
            setTargetRPM(FLYWHEEL_RPM_MID);
            return midPitch(dist);
        }
        else{
            setTargetRPM(FLYWHEEL_RPM_FAR);
            return farPitch(dist);
        }
    }

    public void setPitch(double pos){
        pitch.setPosition(pos);
    }

    // Math Helpers
    public double closePitch(double dist){ return -0.0231174*dist + 1.55621; }
    public double midPitch(double dist){ return 0.00375*dist+0.0958333; }
    public double farPitch(double dist){ return 0.1; }

    public double correctTurretAngle(double desiredAngle, double max, double min){
        if (desiredAngle > max && desiredAngle-360 > min){
            return desiredAngle - 360;
        }
        if (desiredAngle < min && desiredAngle+360 < max){
            return desiredAngle + 360;
        }
        return Math.min(max, (Math.max(desiredAngle, min)));
    }

    public double angleToTarget(double xPos, double yPos, double heading, boolean isTeamRed){
        double goalX = -66.0;
        double goalY = isTeamRed ? 66.0 : -66.0;
        double dx = goalX - xPos;
        double dy = goalY - yPos;
        double fieldTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        return fieldTargetDeg - heading;
    }
}