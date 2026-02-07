package org.firstinspires.ftc.teamcode.auton.parts;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.Intake;
import org.firstinspires.ftc.teamcode.teleop.Spindexer;

public class AutonSpindexer extends Spindexer {
    public AutonSpindexer(HardwareMap hardwareMap, Intake intake) {
        init(hardwareMap, intake);
    }

    public Action startSpindexer() {
        return packet -> {
            update(false, 0, false);
            packet.put("Spindexer Status", "On");
            return false;
        };
    }

    public Action intake() {
        return packet -> {
            setState(SpindexerState.INTAKING);
            packet.put("Spindexer Status", "Indexing");
            return false;
        };
    }
}
