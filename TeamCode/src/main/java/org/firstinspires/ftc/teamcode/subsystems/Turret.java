package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    //--- Flywheel Encoder Constants ---
    private final double TICK_PER_ROTATION = 28.0;
    private final double SECOND_PER_MINUTE = 60;
    private final double RPM_PER_TPS = SECOND_PER_MINUTE/TICK_PER_ROTATION;
    //(NOT DONE)--- Turret Encoder Constants ---
    private final double TICK_PER_DEGREE = 1;
    //--- What is used ---
    private boolean useFlywheelPID = true;
    private boolean useTurretPID = true;

    private boolean useAutoPower = true;
    //--- PIDS ---
    private PID flywheelPID = new PID(0,0,0,0);
    private PID turretPID = new PID(0,0,0,0);
    //--- Constructor ---
    public Turret(HardwareMap hardwareMap){
        DcMotorEx  fw1 = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1 = new CachedMotor(fw1);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setCachingThreshold(0.01);

        DcMotorEx fw2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2 = new CachedMotor(fw2);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setCachingThreshold(0.01);

        DcMotorEx t = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new CachedMotor(t);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setCachingThreshold(0.01);

        pitch = hardwareMap.get(Servo.class, "pitch");
    }
    //--- Main Loop Function ---
    public void update(double dist){
        if(useAutoPower) {
            pitchPosition = autoPitch(dist);
            targetRPM = autoRPM(dist);
        }
        //run PIDS
        if(useFlywheelPID) {
            double flywheelPower = flywheelPID.update(targetRPM, getFlywheelRPM());
            setFlywheelPower(flywheelPower);
        }
        if(useTurretPID) {
            double turretPower = turretPID.update(targetAngle, getTurretAngle());
            setTurretPower(turretPower);
        }
        //set pitch
        pitch.setPosition(pitchPosition);
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
    //--- Setters ---
    public void setUseAutoPower(boolean useAutoPower) {
        this.useAutoPower = useAutoPower;
    }
    public void setUseFlywheelPID(boolean useFlywheelPID) {
        this.useFlywheelPID = useFlywheelPID;
    }
    public void setUseTurretPID(boolean useTurretPID) {
        this.useTurretPID = useTurretPID;
    }
    //--- Helpers ---
    public double getFlywheelRPM(){
        return getFlywheelVelocity() * RPM_PER_TPS;
    }

    public double getTurretAngle(){
        return getTurretPosition()/TICK_PER_DEGREE;
    }
    public boolean isFlywheelReady(){
        return getFlywheelRPM() > targetRPM*0.9;
    }
}
