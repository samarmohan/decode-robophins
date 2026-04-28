package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotTeleop;
@TeleOp(name="Blue TeleOp", group="Competition")
public class BlueTele extends RobotTeleop {
    @Override
    public void init() {
        isTeamRed = false;
        super.init();
    }
}
