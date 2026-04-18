package org.firstinspires.ftc.teamcode.robot;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public abstract class RobotTeleop extends OpMode {
    //--- Systems ---
    protected Robot r;
    protected Follower f;
    protected ElapsedTime runtime = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private double lastTime;
    private boolean distance;
    @Override
    public void init(){
        r = new Robot(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        // TODO set where auton ends
        f.setStartingPose(new Pose(72, 72, 0));
        f.update();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        r.spindexer.alignBack();
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
        spindexer();
        lights();
        tilt();
        turret();
        limelight();
        telemetry();
        lastTime = runtime.seconds();
    }

    public void update(){
        r.clearCache();
        f.update();
    }

    public void drive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double accelerator = gamepad1.left_trigger;

        r.drivetrain.driveFieldCentric(y,x,rx,accelerator);

        if(gamepad1.options){
            r.drivetrain.resetFieldCentric();
        }
    }

    public void intake() {
        if (gamepad1.cross || r.spindexer.isShooting()) {
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
        //r.spindexer.update(gamepad1.cross, gamepad1.right_trigger, true, gamepad2.right_trigger >0.1);
    }
    public void tilt() {
        if (gamepad1.square) {
            r.tilt.toggleTilt();
        }
        r.tilt.update();
    }

    public void limelight() {
        r.limelight.update();
    }

    public void turret() {
        r.turret.updateAutoPower(220);//current just constant
        r.turret.updatePitch(220);
        //r.turret.setPitch(0.5); //0 is min 0.6 is max
        r.turret.updateFlywheelPID();
        //r.turret.updateBlackBox(new Pose(f.getPose().getX(), f.getPose().getY(), f.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid());
    }
    public void telemetry(){
        panelsTelemetry.addData("target RPM", r.turret.getTargetRPM());
        panelsTelemetry.addData("actual RPM", r.turret.getFlywheelRPM());

        telemetry.addData("loop time", runtime.seconds()-lastTime);
        telemetry.addLine("Position: X:"+  f.getPose().getX() + " Y: " +  f.getPose().getY()+ "Heading: " +  f.getPose().getHeading());
        telemetry.addData("Target Flywheel RPM", r.turret.getTargetRPM());
        telemetry.addData("Actual Flywheel RPM", r.turret.getFlywheelRPM());
        telemetry.addData("Flywheel Power", r.turret.getFlywheelPower());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Turret Angle", r.turret.getTurretAngle());
        telemetry.addData("Turret Target Angle", r.turret.getTargetAngle());
        telemetry.addData("Turret Power", r.turret.getTurretPower());
        telemetry.addData("Limelight Valid", r.limelight.wasLastResultValid());
//        telemetry.addLine("------------------------------------");
//        telemetry.addData("Spindexer State", r.spindexer.getState());
//        telemetry.addData("Is Full?", r.spindexer.isFull());
//        telemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
//        telemetry.addData("Target Angle", r.spindexer.getTargetAngle());
//        telemetry.addData("Ball Detected", r.spindexer.ballDetectedSpin());
//        telemetry.addData("Back Distance", r.spindexer.getBackDistance());
        panelsTelemetry.update();
        telemetry.update();
    }
}
