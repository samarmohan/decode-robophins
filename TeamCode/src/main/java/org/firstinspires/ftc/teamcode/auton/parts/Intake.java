package org.firstinspires.ftc.teamcode.auton.parts;

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
        return packet -> {
            frontIntake.setPower(0);
            backIntake.setPower(0);
            packet.put("Intake Status", "Off");
            return false;
        };
    }

    public Action intakeShoot() {
        return packet -> {
            frontIntake.setPower(1);
            backIntake.setPower(1);
            packet.put("Intake Status", "Shooting");
            return false;
        };
    }

    public Action intakeHold() {
        return packet -> {
            frontIntake.setPower(1);
            backIntake.setPower(-1);
            packet.put("Intake Status", "Holding");
            return false;
        };
    }
}