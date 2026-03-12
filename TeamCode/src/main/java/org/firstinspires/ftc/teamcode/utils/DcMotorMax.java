package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class DcMotorMax implements DcMotorEx {
    private double cachingThreshold = 0.0;
    private double lastPower;

    public void setCachingThreshold(double cachingThreshold) {
        this.cachingThreshold = cachingThreshold;
    }

    public double getCachingThreshold() {
        return cachingThreshold;
    }

    @Override
    public void setPower(double power) {
        if (power > lastPower + cachingThreshold && power < lastPower - cachingThreshold){
            setPower(power);
            lastPower = power;
        }
    }
}
