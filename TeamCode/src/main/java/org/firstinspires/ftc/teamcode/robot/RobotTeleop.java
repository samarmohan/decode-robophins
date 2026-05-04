package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.AutonPoseSave;


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
    public boolean isTeamRed;

    boolean turretOn = true;

    double overridePitch = 0;
    boolean tilt = false;
    @Override
    public void init(){
        r = new Robot(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(AutonPoseSave.lastAutonPose);
        f.update();
        r.turret.setTeam(isTeamRed);
        r.limelight.setTeam(isTeamRed);
        telemetry.addData("Team", isTeamRed ? "Red" : "Blue");
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
        if(currentGamepad1.circle){
            f.setPose(isTeamRed ? new Pose(9,9,Math.toRadians(180)) : new Pose(135, 9 ,Math.toRadians(0)));
        }
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
        r.spindexer.update(gamepad1.cross, gamepad1.right_trigger > 0.3, r.turret.isFlywheelReady(), false);
    }
    public void tilt() {
        if (currentGamepad1.square && !previousGamepad1.square) {
            tilt = !tilt;
        }
        if (tilt) {
            r.tilt.tiltDown();
            turretOn = false;
        } else {
            r.tilt.tiltUp();
            turretOn = true;
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
            if (currentGamepad2.left_trigger > 0.2  && currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                r.turret.changeRPMOffset(50);
            }
            else if (currentGamepad2.left_trigger > 0.2  && currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                r.turret.changeRPMOffset(-50);
            }
        } else {
            r.turret.setTargetRPM(0);
        }
        r.turret.updatePitch(r.turret.getDistance(f.getPose()));
        r.turret.updateFlywheelPID();
        r.turret.updateAutoPower(r.turret.getDistance(f.getPose()));//current just constant
        //r.turret.setPitch(0); //0 is min 0.6 is max
        //r.turret.updatePositionAim(f.getPose());
        r.turret.updateBlackBox(new Pose(f.getPose().getX(), f.getPose().getY(), f.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid(), r.turret.getDistance(f.getPose()) > 120);

    }
    public void telemetry() {
        panelsTelemetry.addData("Intake Current", r.intake.intake.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Flywheel1 Current", r.turret.flywheel1.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Flywheel2 Current", r.turret.flywheel2.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Rotation Current", r.turret.turret.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Front Right Current", r.drivetrain.frontRight.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Front Left Current", r.drivetrain.frontLeft.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Back Right Current", r.drivetrain.backRight.motor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Back Left Current", r.drivetrain.backLeft.motor.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Loop Time", runtime.seconds()-lastTime);
        telemetry.addData("Distance", r.turret.getDistance(f.getPose()));
        telemetry.addData("Target RPM", r.turret.getTargetRPM());
        telemetry.addData("Pitch", r.turret.getPitch());

        telemetry.addLine("------------------ODOMETRY------------------");
        telemetry.addLine("Position: X:"+  f.getPose().getX() + " Y: " +  f.getPose().getY()+ "Heading: " +  f.getPose().getHeading());
        telemetry.addData("Distance", r.turret.getDistance(f.getPose()));

        telemetry.addLine("------------------FLYWHEEL------------------");
        telemetry.addData("Target RPM", r.turret.getTargetRPM());
        telemetry.addData("Actual RPM", r.turret.getFlywheelRPM());
        telemetry.addData("Error",r.turret.getFlywheelRPM() - r.turret.getTargetRPM());
        telemetry.addData("Power", r.turret.getFlywheelPower());
        telemetry.addData("RPM Offset", r.turret.getRPMOffset());

        telemetry.addLine("------------------TURRET------------------");
        telemetry.addData("Target Angle", r.turret.getTargetAngle());
        telemetry.addData("Actual Angle", r.turret.getTurretAngle());
        telemetry.addData("Error", r.turret.getTurretAngle() - r.turret.getTurretAngle());
        telemetry.addData("Power", r.turret.getTurretPower());
        telemetry.addData("Pitch", r.turret.getPitch());

        telemetry.addLine("------------------LIMELIGHT------------------");
        telemetry.addData("Tx", r.limelight.getTx());
        telemetry.addData("Valid?", r.limelight.wasLastResultValid());

        telemetry.addLine("------------------SPINDEXER------------------");
        telemetry.addData("State", r.spindexer.getState());
        telemetry.addData("Full?", r.spindexer.isFull());
        telemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
        telemetry.addData("Target Angle", r.spindexer.getTargetAngle());
        telemetry.addData("Ball Detected", r.spindexer.ballDetectedSpin());
        telemetry.addData("Back Distance", r.spindexer.getBackDistance());
        telemetry.addData("Order", r.spindexer.getOrder());

        telemetry.addLine("------------------TILT------------------");
        telemetry.addData("Tilt", r.tilt.getState());
        panelsTelemetry.update();
        telemetry.update();
    }
}
