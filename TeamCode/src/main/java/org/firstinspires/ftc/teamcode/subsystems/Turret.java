package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private DcMotorEx flywheel1, flywheel2, turret;
    private Servo pitch;

    private double targetRPM = 0;

    private double targetAngle = 0;

    private double pitchPosition = 0;

    public Turret(HardwareMap hardwareMap){
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pitch = hardwareMap.get(Servo.class, "pitch");
    }
}
