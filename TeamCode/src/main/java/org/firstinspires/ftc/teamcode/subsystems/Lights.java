package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    //--- Hardware ---
    public Servo lights;// Not actually a servo, but uses PWM power to set color

    //--- States ---
    public enum LightState {
        FULL,
        EMPTY,
        PARTIAL
    }
    public LightState currentLightState = LightState.EMPTY;
    //--- Constructor ---
    public Lights(HardwareMap hardwareMap) {
        lights = hardwareMap.get(Servo.class, "LEDs");
    }
    //--- Main Loop Function ---
    public void update() {
        switch (currentLightState) {
            case FULL:
                setColor(0.5);
                break;
            case EMPTY:
                setColor(0.28);
                break;
            case PARTIAL:
                setColor(0.33);
                break;
        }
    }
    //--- Helpers ---
    public void setLightState(LightState state) {
        currentLightState = state;
    }
    //basic light control for rn
    public void autoLights(boolean hasBall, boolean isFull){
        if(isFull){
            currentLightState = LightState.FULL;
        } else if (hasBall) {
            currentLightState = LightState.PARTIAL;
        }
        else {
            currentLightState = LightState.EMPTY;
        }
    }
    private void setColor(double pwm){
        lights.setPosition(pwm);
    }

}
