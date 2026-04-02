package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    // --- Variables ---
    private boolean distance;
    //--- Opmode Functions ---
    @Override
    public void init(){

        r = new Robot(hardwareMap);
        //f = Constants.createFollower(hardwareMap);
        //f.setStartingPose(new Pose(0, 0, 0));
        //f.update();
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
        update();
        drive();
        intake();
        lights();
        spindexer();
        tilt();
        turret();
        telemetry();
    }

    //--- Subsystems ---
    public void update(){
        r.clearCache();
        r.update();
        //f.update();
    }

    public void drive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double accelerator = gamepad1.left_trigger;

        r.drivetrain.driveFieldCentric(y,x,rx,accelerator);
    }

    public void intake() {
        if (gamepad1.cross) {
            r.intake.intake();
        } else {
            r.intake.outtake();
        }
        r.intake.run();
    }
    public void lights() {
        r.lights.autoLights(r.spindexer.hasBalls(),r.spindexer.isFull());
        r.lights.update();
    }
    public void spindexer() {
        r.spindexer.update(gamepad1.cross, gamepad1.right_trigger, r.turret.isFlywheelReady(), gamepad2.right_trigger >0.1);
    }
    public void tilt() {
        if (gamepad1.square) {
            r.tilt.toggleTilt();
        }
        r.tilt.update();
    }
    public void turret() {
        r.turret.updateAutoPower(220);//current just constant
        r.turret.updatePitch(220);
        r.turret.updateFlywheelPID();
        //r.turret.updatePositionAim(new Pose2D(0,0),0);
    }
    public void telemetry(){
        //telemetry.addData("Position", f.getPose().toString());
        telemetry.addData("Target Flywheel RPM", r.turret.getTargetRPM());
        telemetry.addData("Actual Flywheel RPM", r.turret.getFlywheelRPM());
        telemetry.addData("Flywheel Power", r.turret.getFlywheelPower());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Turret Angle", r.turret.getTurretAngle());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Spindexer State", r.spindexer.getState());
        telemetry.addData("Is Full?", r.spindexer.isFull());
        telemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
        telemetry.addData("Target Angle", r.spindexer.getTargetAngle());
        telemetry.addData("Ball Detected", r.spindexer.ballDetectedSpin());
        telemetry.addData("Back Distance", r.spindexer.getBackDistance());
        telemetry.update();
    }
}
