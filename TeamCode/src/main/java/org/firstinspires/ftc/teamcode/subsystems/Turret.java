package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.TeleOpConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;
import org.firstinspires.ftc.teamcode.utils.PID;

public class Turret {
    //--- Hardware ---
    private CachedMotor flywheel1, flywheel2, turret;
    private Servo pitch1;
    private Servo pitch2;
    //--- Variables --
    private double targetRPM = 0;
    double lastRPM = 0;
    private double targetAngle = 0;
    private double pitchPosition = 0;
    private boolean isTeamRed =  true;
    private Pose redGoalPose = new Pose(144, 144);
    private Pose blueGoalPose = new Pose(0, 144);
    //--- Flywheel Encoder Constants ---
    private final double TICK_PER_ROTATION = 28.0;
    private final double SECOND_PER_MINUTE = 60;
    private final double RPM_PER_TPS = SECOND_PER_MINUTE/TICK_PER_ROTATION;
    //(NOT DONE)--- Turret Encoder Constants ---
    private final double TICK_PER_DEGREE = -8.0111;
    private final double MAX_TURRET_ANGLE = 140;
    private final double MIN_TURRET_ANGLE = -140;
    //--- What is used ---
    //--- PIDS ---
    private final PID flywheelPID = new PID(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD, FLYWHEEL_kF, 0);
    private final PID rotationAnglePID = new PID(ROTATION_ANGLE_kP, ROTATION_ANGLE_kI, ROTATION_ANGLE_kD,0, 0);
    private final PID rotationLimelightPID = new PID(ROTATION_LIMELIGHT_kP, ROTATION_LIMELIGHT_kI, ROTATION_LIMELIGHT_kD, ROTATION_LIMELIGHT_kF, ROTATION_LIMELIGHT_FF_DEADZONE);

    double flywheelPower = 0;
    double turretPower = 0;
    boolean bangBang = false;
    //--- Constructor ---
    public Turret(HardwareMap hardwareMap){
        flywheelPID.setFlywheel(true);

        DcMotorEx fw1 = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = new CachedMotor(fw1);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setCachingThreshold(0.01);

        DcMotorEx fw2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2 = new CachedMotor(fw2);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setCachingThreshold(0.01);

        DcMotorEx t = hardwareMap.get(DcMotorEx.class, "turretTurn");
        turret = new CachedMotor(t);
        turret.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setCachingThreshold(0.01);

        pitch1 = hardwareMap.get(Servo.class, "pitch1");
        pitch2 = hardwareMap.get(Servo.class, "pitch2");
    }
    //--- Main Loop Functions ---
    public void updateAutoPower(double dist) {
        targetRPM = autoRPM(dist);
    }
    public void updateFlywheelPID () {
        double currentRPM = getFlywheelRPM();
        //bang bang for shooting
        if(!bangBang) {
            if (targetRPM > 1500 && lastRPM - currentRPM > 50) {
                bangBang = true;
            }
            flywheelPower = flywheelPID.update(targetRPM, currentRPM);
        }else {
            if (currentRPM >= targetRPM) {
                bangBang = false;
            }
            flywheelPower = 1;
        }
        setFlywheelPower(flywheelPower);
        lastRPM = currentRPM;
    }
    public void updatePitch(double dist){
        double pitchComp;
        double kP = 0.002;
        if(bangBang){
            pitchComp = kP * (lastRPM - getFlywheelRPM());
        }
        else {
            pitchComp = 0;
        }
        pitchPosition = autoPitch(dist) - pitchComp;
        setPitch(pitchPosition);
    }
    // --- Auto-Aim ---
    //-- Black Box --
    public void updateBlackBox(Pose pose, double tx, boolean islimelightValid){
        if (islimelightValid && isWithinBounds(getTurretAngle())) {
            turretPower = rotationLimelightPID.update(0, tx);
            setTurretPower(turretPower);
        }
        else {
            updatePositionAim(pose);
        }
    }
    //-- Position Based --
    public void updatePositionAim(Pose pose){
        targetAngle = correctTurretAngleToGoal(pose, isTeamRed ? redGoalPose : blueGoalPose);
        double turretPower = rotationAnglePID.update(targetAngle, getTurretAngle());
        setTurretPower(turretPower);
    }
    // --- Auto Calculations ---
    public double autoRPM(double dist) {

        if (dist < 120) return 2200;
        return 3100;
    }
    public double autoPitch(double dist) {
        if (dist < 120) return 0.56;
        return 0.67; // Far pitch
    }
    //--- Hardware Interactions ---
    public double getFlywheelVelocity(){
        return flywheel1.getVelocity();
    }
    public double getTurretPosition(){
        return turret.getCurrentPosition();
    }
    public void setFlywheelPower(double power){
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }
    public void setTurretPower(double power){
        turret.setPower(power);
    }
    public double getTurretPower(){
        return turret.getPower();
    }
    public void setPitch(double pos){
        pitch1.setPosition(pos);
        pitch2.setPosition(1 - pos);
    }
    //--- Helpers ---
    public double getFlywheelRPM(){
        return getFlywheelVelocity() * RPM_PER_TPS;
    }

    public double getFlywheelPower() {return flywheelPower;}
    public double getTargetRPM() {
        return targetRPM;
    }
    public void setTargetRPM(double RPM){
        targetRPM = RPM;
    }
    public void setTeam(boolean red){
        isTeamRed = red;
    }
    public double getTurretAngle(){
        return getTurretPosition()/TICK_PER_DEGREE;
    }
    public boolean isFlywheelReady(){
        return getFlywheelRPM() > targetRPM*0.9;
    }
    public double getTargetAngle(){
        return targetAngle;
    }
    public double correctTurretAngleToGoal(Pose pose, Pose goalPose){
        double xToGoal = goalPose.getX() - pose.getX();
        double yToGoal = goalPose.getY() - pose.getY();
        double robotHeading = Math.toDegrees(pose.getHeading());
        double headingToGoal = Math.toDegrees(Math.atan2(yToGoal, xToGoal));
        double targetTurretAngle = headingToGoal - robotHeading;
        double normalizedAngle = Math.IEEEremainder(targetTurretAngle, 360);
        if (isWithinBounds(normalizedAngle)){
            return normalizedAngle;
        }
        double distToMax = Math.abs(normalizedAngle - MAX_TURRET_ANGLE);
        double distToMin = Math.abs(normalizedAngle - MIN_TURRET_ANGLE);

        return distToMax <= distToMin ? MAX_TURRET_ANGLE : MIN_TURRET_ANGLE;
    }
    public double getPitch(){
        return pitchPosition;
    }
    public boolean getBangBang(){
        return bangBang;
    }
    public boolean isWithinBounds(double angle){
        return angle > MIN_TURRET_ANGLE && angle < MAX_TURRET_ANGLE;
    }
    public double getDistance(Pose pose){
        Pose goalPose = isTeamRed ? redGoalPose : blueGoalPose;
        return Math.sqrt(Math.pow(goalPose.getX()-pose.getX(), 2) + Math.pow(goalPose.getY()-pose.getY(), 2));
    }
}
