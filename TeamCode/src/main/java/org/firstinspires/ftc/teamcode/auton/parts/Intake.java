package org.firstinspires.ftc.teamcode.auton.parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor frontIntake;
    private final DcMotor backIntake;

    public Intake(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        //positive is in, negative is out
        frontIntake.setDirection(DcMotor.Direction.REVERSE);
        //positive is up, negative is down
        backIntake.setDirection(DcMotor.Direction.FORWARD);
    }

    public Action intakeOff() {
        return new IntakeOff();
    }

    public Action intakeShoot() {
        return new IntakeShoot();
    }

    public Action intakeHold() {
        return new IntakeHold();
    }

    public class IntakeOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontIntake.setPower(0);
            backIntake.setPower(0);
            return false;
        }
    }

    public class IntakeShoot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontIntake.setPower(1);
            backIntake.setPower(1);
            return false;
        }
    }

    public class IntakeHold implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontIntake.setPower(1);
            backIntake.setPower(-1);
            return false;
        }
    }
}