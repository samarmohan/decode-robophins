package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotTeleop;

@TeleOp(name="Red TeleOp", group="Competition")
public class RedTele extends RobotTeleop {
    @Override
    public void init() {
        isTeamRed = true;
        super.init();
    }
}
