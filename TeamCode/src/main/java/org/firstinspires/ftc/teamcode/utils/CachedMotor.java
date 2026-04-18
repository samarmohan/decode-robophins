package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedMotor{
    private final DcMotorEx motor;
    private double cachingThreshold = 0.001;
    private double lastPower = 0.0;

    public CachedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setCachingThreshold(double cachingThreshold) {
        this.cachingThreshold = cachingThreshold;
    }


    public void setPower(double power) {
        if (Math.abs(power - lastPower) > cachingThreshold) {
            motor.setPower(power);
            lastPower = power;
        }
    }
    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return motor.getPower();
    }
    public double getVelocity(){
        return motor.getVelocity();
    }
    }