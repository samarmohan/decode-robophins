package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor frontIntake;


    public enum State {
        INTAKE,
        OUTTAKE,
        SHOOTING,
        OFF
    }

    private State currentState = State.OFF;

    public void init(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotor.class, "intake");

        frontIntake.setDirection(DcMotor.Direction.REVERSE); // Positive = In
    }

    /**
     * Updates intake state based on driver inputs.
     * Buttons: Cross (In), Square (Out), Triangle (Off). Trigger (Shoot).
     * Logic: Shoot only if INTAKE mode active + RPM ready. Return to INTAKE after shooting.
     */
    public void update(boolean in, boolean out, boolean off, double shootTrigger, boolean isFlywheelReady) {

        // 1. Emergency/Stop Override
        if (off) {
            currentState = State.OFF;
        }
        // 2. Shooting Logic
        else if (shootTrigger > 0.2) {
            // Can only shoot if we are currently intaking (holding a ball) AND flywheel is fast enough
            if (currentState == State.INTAKE && isFlywheelReady) {
                currentState = State.SHOOTING;
            }
            // If in OFF or OUTTAKE, trigger does nothing
        }
        // 3. Return from Shooting
        else if (currentState == State.SHOOTING && shootTrigger < 0.2) {
            // Trigger released, go back to grabbing the next ball
            currentState = State.INTAKE;
        }
        // 4. Selector Buttons (Latching)
        else if (in) {
            currentState = State.INTAKE;
        }
        else if (out) {
            currentState = State.OUTTAKE;
        }

        // 5. Apply Motor Powers
        switch (currentState) {
            case INTAKE:
                frontIntake.setPower(1.0);
                break;
            case OUTTAKE:
                frontIntake.setPower(-1.0);
                break;
            case SHOOTING:
                frontIntake.setPower(1.0);
                break;
            case OFF:
                frontIntake.setPower(0.0);
                break;
        }
    }

    public State getState() {
        return currentState;
    }
}