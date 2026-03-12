package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.DcMotorMax;

public class Intake {
    private final DcMotorMax intake;

    public enum IntakeState {
        INTAKE,
        SLOW_INTAKE,
        OUTTAKE,
        OFF
    }

    private IntakeState currentIntakeState = IntakeState.OFF;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorMax.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setCachingThreshold(0.3);
    }

    public void setIntakeState(IntakeState state) {
        currentIntakeState = state;
    }
    //basic on off intaking
    public void intake(){
        setIntakeState(IntakeState.INTAKE);
    }
    public void slowIntake(){
        setIntakeState(IntakeState.SLOW_INTAKE);
    }
    public void outtake(){
        setIntakeState(IntakeState.OUTTAKE);
    }
    public void turnIntakeOff(){
        setIntakeState(IntakeState.OFF);
    }

    public void run() {
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
            }
    }

}
