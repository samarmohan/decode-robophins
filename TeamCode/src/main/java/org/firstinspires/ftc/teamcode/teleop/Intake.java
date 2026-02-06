package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;

    public enum State {
        INTAKE,
        OUTTAKE,
        OFF
    }

    private State currentState = State.OFF;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        // Positive = In
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setState(State state) {
        currentState = state;

        switch (currentState) {
            case INTAKE:
                intake.setPower(1.0);
                break;
            case OUTTAKE:
                intake.setPower(-1.0);
                break;
            case OFF:
                intake.setPower(0.0);
                break;
        }
    }

    public State getState() {
        return currentState;
    }
}