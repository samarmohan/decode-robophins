package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intake;

    public enum IntakeState {
        INTAKE,
        SLOW_INTAKE,
        OUTTAKE,
        OFF
    }

    private IntakeState previousIntakeState = IntakeState.OFF;
    private IntakeState currentIntakeState = IntakeState.OFF;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setIntakeState(IntakeState state) {
        currentIntakeState = state;
    }

    public void run() {
        // Power only changes when state changes, reducing loop times.
        if (currentIntakeState != previousIntakeState) {
            switch (currentIntakeState) {
                case INTAKE:
                    intake.setPower(1);
                    break;
                case SLOW_INTAKE:
                    intake.setPower(0.5);
                    break;
                case OUTTAKE:
                    intake.setPower(-1.0);
                    break;
                case OFF:
                    intake.setPower(0.0);
                    break;
            };
            previousIntakeState = currentIntakeState;
        }
    }

}
