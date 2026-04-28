package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotTeleop;

@TeleOp(name="Drive TeleOp", group="")
@Disabled
public class DriveTeleOp extends OpMode {
    protected Robot r;
    protected Follower f;
    protected ElapsedTime runtime = new ElapsedTime();
    private double lastTime;
    private boolean distance;
    @Override
    public void init(){
        r = new Robot(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(72, 72, 0));
        f.update();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }
    @Override
    public void start() {
        telemetry.setMsTransmissionInterval(10);
        runtime.reset();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double accelerator = gamepad1.left_trigger;

        r.drivetrain.driveFieldCentric(y,x,rx,accelerator);

        if(gamepad1.options){
            r.drivetrain.resetFieldCentric();
        }
    }
}
