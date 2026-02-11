package org.firstinspires.ftc.teamcode.auton.parts;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Turret;

public class AutonTurret extends Turret {

    public AutonTurret(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public Action setFlywheelRPM(double rpm) {
        return packet -> {
            setTargetRPM(rpm);
            double currentTimeSeconds = System.nanoTime() / 1e9;
            updateFlywheelPID(currentTimeSeconds);
            applyFlywheelPower();
            packet.put("Flywheel Target RPM", rpm);
            packet.put("Flywheel output", getFlywheelRPM());
            return true;
        };
    }

    public Action setPitchPosition(double pitchPos) {
        return packet -> {
            setPitch(pitchPos);
            packet.put("Pitch Position", pitchPos);
            return false;
        };
    }
}