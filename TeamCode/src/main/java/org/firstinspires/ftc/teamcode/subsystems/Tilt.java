package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tilt {
    private Servo tilt;

    public enum TiltState {
        UP,
        DOWN
    }

    private TiltState previousTiltState = TiltState.UP;
    private TiltState currentTiltState = TiltState.UP;

    public Tilt(HardwareMap hardwareMap){
        tilt = hardwareMap.get(Servo.class, "tilt");
    }

    public void setTiltState(TiltState state){
        currentTiltState = state;
    }

    public void update(){
        if (currentTiltState != previousTiltState) {
            switch (currentTiltState) {
                case UP:
                    tilt.setPosition(0.7);
                    break;
                case DOWN:
                    tilt.setPosition(1.0);
                    break;
            }
        }
    }
}
