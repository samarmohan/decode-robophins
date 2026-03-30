package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;

public class Intake {
    //--- Hardware ---
    private CachedMotor intake;  // Changed from DcMotorEx to CachedMotor

    //--- States ---
    public enum IntakeState {
        INTAKE,
        SLOW_INTAKE,
        OUTTAKE,
        OFF
    }
    private IntakeState currentIntakeState = IntakeState.OFF;

    //--- Constructor ---
    public Intake(HardwareMap hardwareMap) {
        // Get the raw motor from hardware map
        DcMotorEx rawMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Wrap it in CachedMotor
        intake = new CachedMotor(rawMotor);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setCachingThreshold(0.3);
    }

    //--- Main Loop Function ---
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

    //--- Helpers ---
    public void setIntakeState(IntakeState state) {
        currentIntakeState = state;
    }

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
}