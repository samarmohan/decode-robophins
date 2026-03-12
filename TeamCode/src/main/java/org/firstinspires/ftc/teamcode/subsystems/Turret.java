package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.DcMotorMax;
import org.firstinspires.ftc.teamcode.utils.PID;

public class Turret {
    private DcMotorMax flywheel1, flywheel2, turret;
    private Servo pitch;

    private double targetRPM = 0;

    private double targetAngle = 0;

    private double pitchPosition = 0;

    private PID flywheelPID = new PID(0,0,0,0);
    private PID turretPID = new PID(0,0,0,0);

    public Turret(HardwareMap hardwareMap){
        flywheel1 = hardwareMap.get(DcMotorMax.class, "flywheel");
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setCachingThreshold(0.01);

        flywheel2 = hardwareMap.get(DcMotorMax.class, "flywheel2");
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setCachingThreshold(0.01);

        turret = hardwareMap.get(DcMotorMax.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setCachingThreshold(0.01);

        pitch = hardwareMap.get(Servo.class, "pitch");
    }
    public boolean isFlywheelReady(){
        //not done yet
        return true;
    }

    public void update(){

    }

}
