package org.firstinspires.ftc.teamcode.auton.parts;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teleop.Turret;

public class AutonTurret extends Turret {

    public AutonTurret(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public Action setFlywheelRPM(double rpm, double currentTimeSeconds) {
        return packet -> {
            setTargetRPM(rpm);
            updateFlywheelPID(currentTimeSeconds);
            applyFlywheelPower();
            packet.put("Flywheel Target RPM", rpm);
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