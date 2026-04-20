package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotTeleop;

@TeleOp(name = "Regression TeleOp", group = "!!")
public class RegressionTeleOp extends RobotTeleop {
    private double pitch = 0;
    private double RPM = 0;
    @Override
    public void turret(){
        if(gamepad2.left_bumper){
            RPM -= 100;
        }
        if(gamepad2.right_bumper){
            RPM += 100;
        }
        if(gamepad2.a){
            pitch += 0.01;
        }
        if(gamepad2.b){
            pitch -= 0.01;
        }
        r.turret.setPitch(pitch);
        r.turret.setTargetRPM(RPM);
        r.turret.updateFlywheelPID();
        r.turret.updateBlackBox(new Pose(f.getPose().getX(), f.getPose().getY(), f.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid());
    }
    @Override
    public void telemetry(){
        telemetry.addData("Pitch", pitch);
        super.telemetry();
    }
}
