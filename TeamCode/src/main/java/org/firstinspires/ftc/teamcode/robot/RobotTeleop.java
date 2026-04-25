package org.firstinspires.ftc.teamcode.robot;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public abstract class RobotTeleop extends OpMode {
    //--- Systems ---
    protected Robot r;
    protected Follower f;
    protected ElapsedTime runtime = new ElapsedTime();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();

    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();
    private double lastTime;
    private boolean distance;

    boolean turretOn = true;
    @Override
    public void init(){
        r = new Robot(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        // TODO set where auton ends
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
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
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
        r.intake.setIntakeState(r.spindexer.getIntakeState());
        /*
        if (gamepad1.cross || r.spindexer.isShooting()) {
            r.intake.intake();
        } else {
            r.intake.outtake();
        }
         */
        r.intake.run();
    }
    public void lights() {
        r.lights.autoLights(r.spindexer.hasBalls(),r.spindexer.isFull());
        r.lights.update();
    }
    public void spindexer() {
        r.spindexer.update(gamepad1.cross, gamepad1.right_trigger > 0.3, true, gamepad2.right_trigger >0.1);
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
        if (gamepad1.left_bumper) {
            turretOn = false;
        } else if (gamepad1.right_bumper) {
            turretOn = true;
        }
        if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
            r.turret.changeOffset(2.5);
        }
        else if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            r.turret.changeOffset(-2.5);
        }
        if (turretOn) {
            r.turret.updateAutoPower(r.turret.getDistance(f.getPose()));//current just constant
        } else {
            r.turret.setTargetRPM(0);
        }
        r.turret.updateFlywheelPID();
        r.turret.updatePitch(r.turret.getDistance(f.getPose()));
        //r.turret.setPitch(0); //0 is min 0.6 is max
        //r.turret.updatePositionAim(f.getPose());
        r.turret.updateBlackBox(new Pose(f.getPose().getX(), f.getPose().getY(), f.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid());

    }
    public void telemetry(){
        panelsTelemetry.addData("target RPM", r.turret.getTargetRPM());
        panelsTelemetry.addData("actual RPM", r.turret.getFlywheelRPM());
        panelsTelemetry.addData("flywheel power", r.turret.getFlywheelPower());
        panelsTelemetry.addData("Pitch Postion", r.turret.getPitch());
        panelsTelemetry.addData("error",r.turret.getFlywheelRPM() - r.turret.getTargetRPM());
        telemetry.addData("loop time", runtime.seconds()-lastTime);
        telemetry.addLine("Position: X:"+  f.getPose().getX() + " Y: " +  f.getPose().getY()+ "Heading: " +  f.getPose().getHeading());
        telemetry.addData("Target Flywheel RPM", r.turret.getTargetRPM());
        telemetry.addData("Actual Flywheel RPM", r.turret.getFlywheelRPM());
        telemetry.addData("Flywheel Power", r.turret.getFlywheelPower());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Turret Angle", r.turret.getTurretAngle());
        telemetry.addData("Turret Target Angle", r.turret.getTargetAngle());
        telemetry.addData("Turret Power", r.turret.getTurretPower());
        telemetry.addData("Distance", r.turret.getDistance(f.getPose()));
        telemetry.addData("is running Bang Bang", r.turret.getBangBang());
        telemetry.addData("Limelight error", r.limelight.getTx());
        telemetry.addData("Limelight Valid", r.limelight.wasLastResultValid());
        telemetry.addData("Target Pitch", r.turret.getPitch());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Spindexer State", r.spindexer.getState());
        telemetry.addData("Is Full?", r.spindexer.isFull());
        telemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
        telemetry.addData("Target Angle", r.spindexer.getTargetAngle());
        telemetry.addData("Ball Detected", r.spindexer.ballDetectedSpin());
        telemetry.addData("Back Distance", r.spindexer.getBackDistance());
        telemetry.addData("order", r.spindexer.getOrder());
        panelsTelemetry.update();
        telemetry.update();
    }
}
