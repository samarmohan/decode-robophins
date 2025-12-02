package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor frontIntake;
    private DcMotor backIntake;

    public void init(HardwareMap hardwareMap){
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        //positive is in, negative is out
        frontIntake.setDirection(DcMotor.Direction.REVERSE);
        //positive is up, negative is down
        backIntake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void in(){
        frontIntake.setPower(1.0);
        backIntake.setPower(-1.0);
    }
    public void out(){
        frontIntake.setPower(-1.0);
        backIntake.setPower(-1.0);
    }
    public void off(){
        frontIntake.setPower(0.0);
        backIntake.setPower(0.0);
    }
    public void shoot(){
        frontIntake.setPower(1.0);
        backIntake.setPower(1.0);
    }
}
