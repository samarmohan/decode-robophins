package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.CachedMotor;
import org.firstinspires.ftc.teamcode.utils.PID;

public class Turret {
    //--- Hardware ---
    private CachedMotor flywheel1, flywheel2, turret;
    private Servo pitch;
    //--- Variables --
    private double targetRPM = 0;
    private double targetAngle = 0;
    private double pitchPosition = 0;
    private Pose goalPose;
    //--- Flywheel Encoder Constants ---
    private final double TICK_PER_ROTATION = 28.0;
    private final double SECOND_PER_MINUTE = 60;
    private final double RPM_PER_TPS = SECOND_PER_MINUTE/TICK_PER_ROTATION;
    //(NOT DONE)--- Turret Encoder Constants ---
    private final double TICK_PER_DEGREE = 1;
    private final double MAX_TURRET_ANGLE = 180;
    private final double MIN_TURRET_ANGLE = -180;
    //--- What is used ---
    //--- PIDS ---
    private PID flywheelPID = new PID(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD, FLYWHEEL_kF);
    private PID turretPID = new PID(0,0,0,0);
    private PID limelightPID = new PID(0,0,0,0);
    //--- Constructor ---
    public Turret(HardwareMap hardwareMap, boolean isTeamRed){
        DcMotorEx  fw1 = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = new CachedMotor(fw1);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setCachingThreshold(0.01);

        DcMotorEx fw2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2 = new CachedMotor(fw2);
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setCachingThreshold(0.01);

        DcMotorEx t = hardwareMap.get(DcMotorEx.class, "turretTurn");
        turret = new CachedMotor(t);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setCachingThreshold(0.01);

        pitch = hardwareMap.get(Servo.class, "pitch");

        if(isTeamRed){
            goalPose = new Pose(-72, 72);
        }else {
            goalPose = new Pose(-72, -72);
        }
    }
    //--- Main Loop Functions ---
    public void updateAutoPower(double dist){
        pitchPosition = autoPitch(dist);
        targetRPM = autoRPM(dist);
    }
    public void updateFlywheelPID(){
        double flywheelPower = flywheelPID.update(targetRPM, getFlywheelRPM());
        setFlywheelPower(flywheelPower);
    }
    public void updatePitch(double dist){
        pitch.setPosition(pitchPosition);
    }
    // --- Auto-Aim ---
    //-- Black Box --
    public void updateBlackBox(Pose pose, double heading, double ty, boolean islimelightValid){
        if(islimelightValid){
            double turretPower = limelightPID.update(ty, 0);
            setTurretPower(turretPower);
        }
        else{
            targetAngle = correctTurretAngleToGoal(pose, goalPose, heading);
            double turretPower = turretPID.update(targetAngle, getTurretAngle());
        }
    }
    //-- Position Based --
    public void updatePositionAim(Pose pose, double heading){
        targetAngle = correctTurretAngleToGoal(pose, goalPose, heading);
        double turretPower = turretPID.update(targetAngle, getTurretAngle());
    }
    // --- Auto Calculations ---
    public double autoRPM(double dist) {

        if (dist < 30) return 2200;
        if (dist < 225) return 4 * dist + 1920;
        return 5 * dist + 1525;
    }
    public double autoPitch(double dist) {
        if (dist < 30) return 0.3;
        if (dist < 260) return 0.655;
        return 0.72; // Far pitch
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
    public void setPitch(double pos){
        pitch.setPosition(pos);
    }
    //--- Helpers ---
    public double getFlywheelRPM(){
        return getFlywheelVelocity() * RPM_PER_TPS;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
    public double getTurretAngle(){
        return getTurretPosition()/TICK_PER_DEGREE;
    }
    public boolean isFlywheelReady(){
        return getFlywheelRPM() > targetRPM*0.9;
    }
    public double correctTurretAngleToGoal(Pose pose, Pose goalPose, double heading){
        double xToGoal = goalPose.getX() - pose.getX();
        double yToGoal = goalPose.getY() - pose.getY();
        double robotHeading = Math.toDegrees(heading);
        double headingToGoal = Math.toDegrees(Math.atan2(yToGoal, xToGoal));
        double targetTurretAngle = headingToGoal - robotHeading;
        return Math.IEEEremainder(targetTurretAngle, 360);
    }
}
