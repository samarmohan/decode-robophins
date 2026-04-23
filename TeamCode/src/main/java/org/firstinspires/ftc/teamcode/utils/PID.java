package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

public class PID {
    private double kP, kI, kD, kF, deadzone, integral, lastError, lastTime;
    private boolean flywheel = false;

    public PID(double p, double i, double d, double f, double deadzone) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        this.deadzone = deadzone;
        lastTime = System.nanoTime() / 1e9;
        reset();
    }
    public double update(double target, double current){
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        lastTime = currentTime;

        double error = target - current;
//        if (flywheel && Math.abs(error) < FLYWHEEL_PID_DEADZONE) {
//            error = 0;
//        }
        integral += error * dt;
        integral = Range.clip(integral, -1, 1);


        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = Math.abs(error) > deadzone ? kF * Math.signum(error) : 0;
        if (flywheel) {
            ff = kF;
        }

        double output = ff + (kP * error) + (kI * integral) + (kD * derivative);
        return Range.clip(output, -1, 1);
    }

    public void reset() {
        lastTime = 0;
        integral = 0;
        lastError = 0;
    }

    public void setFlywheel(boolean flywheel){
        this.flywheel = flywheel;
    }

}
