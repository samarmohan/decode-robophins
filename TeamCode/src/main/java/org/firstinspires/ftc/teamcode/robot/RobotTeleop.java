package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dalvik.system.DelegateLastClassLoader;


public abstract class RobotTeleop extends OpMode {
    //--- Systems ---
    protected Robot r;
    protected Follower f;
    // --- Timer ---
    protected ElapsedTime runtime = new ElapsedTime();
    // --- Variables for child Class ---
    protected boolean isTeamRed = false;
    // --- Variables ---
    private boolean distance;
    //--- Opmode Functions ---
    @Override
    public void init(){
        try {
            r = new Robot(hardwareMap, isTeamRed);
            f = Constants.createFollower(hardwareMap);
            f.setStartingPose(new Pose(0, 0, 0));
            f.update();
            telemetry.addData("Status", "Init Complete");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addData("Stack", e.getStackTrace()[0].toString());
        }
        telemetry.update();
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void loop(){
        update();
        drive();
        intake();
        lights();
        spindexer();
        tilt();
        turret();
        telemetry();
    }
    @Override
    public void stop(){

    }
    //--- Subsystems ---
    public void update(){
        r.clearCache();
        r.update();
        f.update();
    }

    public void drive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double accelerator = gamepad1.left_trigger;

        r.drivetrain.driveFieldCentric(y,x,rx,accelerator);
    }

    public void intake(){
        if(gamepad1.cross){
            r.intake.intake();
        }else{
            r.intake.outtake();
        }
        r.intake.run();
    }
    public void lights(){
        r.lights.autoLights(r.spindexer.hasBalls(),r.spindexer.isFull());
        r.lights.update();
    }
    public void spindexer(){
        r.spindexer.update(gamepad1.cross, gamepad1.right_trigger, r.turret.isFlywheelReady(), gamepad2.right_trigger >0.1);
    }
    public void tilt(){
        if (gamepad1.square){
            r.tilt.toggleTilt();
        }
        r.tilt.update();
    }
    public void turret() {
        r.turret.updateAutoPower(100);//current just constant
        r.turret.updatePitch(100);
        r.turret.updateFlywheelPID();
        r.turret.updatePositionAim(f.getPose(),f.getHeading());
    }
    public void telemetry(){
        telemetry.addData("Position", f.getPose().toString());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Spindexer State", r.spindexer.getState().toString());
        telemetry.addData("back Sensor distance", r.spindexer.getBackDistance());
        telemetry.addData("spindexer target angle", r.spindexer.getTargetAngle());
        telemetry.addData("spindexer current angle", r.spindexer.getCurrentAngle());
        telemetry.addLine("-----------------------------------");
        telemetry.addData("RPM", r.turret.getFlywheelRPM());
        telemetry.addData("turret angle", r.turret.getTurretAngle());

        telemetry.update();
    }

}
