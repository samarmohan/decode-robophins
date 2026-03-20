package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    //--- Hardware ---
    private Servo tilt;
    //--- States ---
    public enum TiltState {
        UP,
        DOWN
    }
    private TiltState currentTiltState = TiltState.UP;
    //--- Constructor ---
    public Tilt(HardwareMap hardwareMap){
        tilt = hardwareMap.get(Servo.class, "tilt");
    }
    //--- Main Function ---
    public void update(){
        switch (currentTiltState) {
            case UP:
                tilt.setPosition(0.7);
                break;
            case DOWN:
                tilt.setPosition(1.0);
                break;
        }
    }
    //--- Helpers ---
    public void setTiltState(TiltState state){
        currentTiltState = state;
    }
    //teleop set state
    public void toggleTilt(){
            if (currentTiltState == TiltState.UP){
                currentTiltState = TiltState.DOWN;
            } else if (currentTiltState == TiltState.DOWN) {
                currentTiltState = TiltState.UP;
            }
    }
}
