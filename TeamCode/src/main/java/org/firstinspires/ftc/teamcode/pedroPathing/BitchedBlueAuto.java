package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.utils.AutonPoseSave;

@Autonomous(name = "Get Nuked by Exodus", group = "Test")
public class BitchedBlueAuto extends OpMode {
    private Follower follower;
    private Robot r;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime shootTimer;
    private String pathState;
    //points
    private double INTAKE_SPEED = 0.7;
    private final Pose startPose       = new Pose(32, 134, Math.toRadians(180));
    private final Pose shootPose       = new Pose(63, 130,  Math.toRadians(180));

    //variables
    private boolean shouldIntake = false;
    private boolean shouldShoot = false;
    private boolean subsystemsOn = true;
    private boolean hasResetTimer = false;

    //loop functions
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new ElapsedTime();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        r = new Robot(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("startToShoot");
        r.turret.setTeam(false);
        r.spindexer.setOrder(2,1,1);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        r.clearCache();
        autonomousPathUpdate();
        intake();
        limelight();
        spindexer();
        turret();
        telemetry();
    }

    private PathChain startToShoot;       // start -> shoot

    public void buildPaths() {

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }


    //Finite State Machine
    public void autonomousPathUpdate() {
        if(opmodeTimer.getElapsedTimeSeconds() > 25) {
            switch (pathState) {
                case "startToShoot":
                    follower.followPath(startToShoot);
                    setPathState("shootFirst");
                    break;
                case "shootFirst":
                    if (!follower.isBusy()) {
                        shouldShoot = true;
                        if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                            shouldShoot = false;
                            subsystemsOn = false;
                            setPathState("end");
                        }
                    }
                    break;
                case "end":
                    AutonPoseSave.lastAutonPose = follower.getPose();
                    break;
            }
        }
    }
    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    //robot subsystems
    private void intake(){
        if(shouldIntake && !r.spindexer.isFull()){
            r.intake.intake();
        }else if(!subsystemsOn){
            r.intake.turnIntakeOff();
        }
        else{
            r.intake.outtake();
        }
        r.intake.run();
    }
    private void limelight(){
        r.limelight.update();
    }
    private void spindexer() {
        if(subsystemsOn){
            r.spindexer.update(shouldIntake, shouldShoot, r.turret.isFlywheelReady(),false);
        }else{
            r.spindexer.setTargetAngle(240);
        }
    }
    private void turret() {
        if(subsystemsOn) {
            r.turret.updateAutoPower(r.turret.getDistance(follower.getPose()));
            r.turret.updatePitch(r.turret.getDistance(follower.getPose()));
            r.turret.updateFlywheelPID();
            r.turret.updatePositionAim(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()));
        }
        else{
            r.turret.setTargetRPM(0);
            r.turret.updateFlywheelPID();
            r.turret.aimForward();
        }
    }
    public void telemetry(){
        // Feedback to Driver Hub for debugging\
        panelsTelemetry.addData("isShooting var", r.spindexer.isShooting() ? 1 : 0);
        panelsTelemetry.addData("hasBalls var", r.spindexer.hasBalls()? 1 : 0);
        panelsTelemetry.addData("is Within Tolerance?", r.spindexer.isWithinTolerance(r.spindexer.getCurrentAngle(), r.spindexer.getTargetAngle()) ? 1:0);
        panelsTelemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
        panelsTelemetry.addData("Target Angle", r.spindexer.getTargetAngle());

        telemetry.addData("isShooting var", r.spindexer.isShooting());
        telemetry.addData("hasBalls var", r.spindexer.hasBalls());
        telemetry.addData("is Within Tolerance?", r.spindexer.isWithinTolerance(r.spindexer.getCurrentAngle(), r.spindexer.getTargetAngle()));
        telemetry.addData("finish shooting", !r.spindexer.isShooting() && !r.spindexer.hasBalls());
        telemetry.addData("path state", pathState);
        telemetry.addData("shoot?",shouldShoot);
        telemetry.addData("intake?", shouldIntake);
        telemetry.addData("Spindexer State", r.spindexer.getState());
        telemetry.addData("Current Angle", r.spindexer.getCurrentAngle());
        telemetry.addData("Target Angle", r.spindexer.getTargetAngle());
        telemetry.addData("Ball Detected", r.spindexer.ballDetectedSpin());
        telemetry.addData("Back Distance", r.spindexer.getBackDistance());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addLine("------------------------------------");
//        telemetry.addData("Target Flywheel RPM", r.turret.getTargetRPM());
//        telemetry.addData("Actual Flywheel RPM", r.turret.getFlywheelRPM());
//        telemetry.addData("Flywheel Power", r.turret.getFlywheelPower());
//        telemetry.addLine("------------------------------------");
//        telemetry.addData("Turret Angle", r.turret.getTurretAngle());
//        telemetry.addData("Turret Target Angle", r.turret.getTargetAngle());
//        telemetry.addData("Turret Power", r.turret.getTurretPower());
//        telemetry.addData("is running Bang Bang", r.turret.getBangBang());
//        telemetry.addData("Limelight Valid", r.limelight.wasLastResultValid());
//        telemetry.addData("Target Pitch", r.turret.getPitch());
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