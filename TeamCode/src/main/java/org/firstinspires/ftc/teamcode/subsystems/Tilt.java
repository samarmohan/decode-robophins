package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    //--- Hardware ---
    private Servo tilt1;
    private Servo tilt2;

    //--- States ---
    public enum TiltState {
        UP,
        DOWN
    }
    private TiltState currentTiltState = TiltState.UP;
    //--- Constructor ---
    public Tilt(HardwareMap hardwareMap){
        tilt1 = hardwareMap.get(Servo.class, "tilt1");
        tilt2 = hardwareMap.get(Servo.class, "tilt2");
    }
    //--- Main Function ---
    public void update() {
        switch (currentTiltState) {
            case UP:
                tilt1.setPosition(0.7);
                tilt2.setPosition(0.7);
                break;
            case DOWN:
                tilt1.setPosition(0.45);
                tilt2.setPosition(0.95);
                break;
        }
    }
    //--- Helpers ---
    public void setTiltState(TiltState state)   {
        currentTiltState = state;
    }
    //teleop set state
    public void tiltUp() {
        if (currentTiltState == TiltState.DOWN) {
            currentTiltState = TiltState.UP;
        }
    }
    public void tiltDown() {
        if (currentTiltState == TiltState.UP) {
            currentTiltState = TiltState.DOWN;
        }
    }

    public TiltState getState() {
        return currentTiltState;
    }
}
