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

@Autonomous(name = "Blue Close Auto", group = "Test")
public class CloseBlueAuto extends OpMode {
    private Follower follower;
    private Robot r;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime shootTimer;
    private String pathState;
    //points
    private double INTAKE_SPEED = 0.7;

    private final Pose startPose       = new Pose(16, 112, Math.toRadians(180));
    private final Pose shootPose       = new Pose(54, 84,  Math.toRadians(180));
    private final Pose collect1stPose  = new Pose(16, 84,  Math.toRadians(180));  // same x/y, adjust if approach differs
    private final Pose lineup2ndPose   = new Pose(54, 60,  Math.toRadians(180));  // pickup2ndSpike
    private final Pose collect2ndPose  = new Pose(16, 60,  Math.toRadians(180));  // collect2ndSpike
    private final Pose lineup3rdPose   = new Pose(42, 36,  Math.toRadians(180));  // pickUp3rdSpike
    private final Pose collect3rdPose  = new Pose(16, 36,  Math.toRadians(180));  // collect3rdSpike
    private final Pose leavePose       = new Pose(56, 50,  Math.toRadians(180));

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

    private PathChain shootToCollect1st;  // shoot -> collect 1st (single segment, no lineup needed)
    private PathChain collect1stToShoot;  // collect 1st -> shoot

    private PathChain shootToCollect2nd;  // shoot -> lineup 2nd -> collect 2nd
    private PathChain collect2ndToShoot;  // collect 2nd -> shoot

    private PathChain shootToCollect3rd;  // shoot -> lineup 3rd -> collect 3rd
    private PathChain collect3rdToShoot;  // collect 3rd -> shoot

    private PathChain shootToLeave;       // shoot -> leave

    public void buildPaths() {

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // ---------- Spike 1 ----------
        // collect1stPose is a direct line from shoot, no separate lineup needed

        shootToCollect1st = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collect1stPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        collect1stToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect1stPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // ---------- Spike 2 ----------

        shootToCollect2nd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineup2ndPose))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(lineup2ndPose, collect2ndPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        collect2ndToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect2ndPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // ---------- Spike 3 ----------

        shootToCollect3rd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineup3rdPose))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(lineup3rdPose, collect3rdPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        collect3rdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collect3rdPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // ---------- Exit ----------

        shootToLeave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }


    //Finite State Machine
    public void autonomousPathUpdate() {
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
                        follower.followPath(shootToCollect1st);
                        setPathState("collect1stSpike");
                    }
                }
                break;
            case "collect1stSpike":
                shouldIntake = true;
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()){
                        hasResetTimer = false;
                        follower.followPath(collect1stToShoot);
                        setPathState("1stSpikeToShoot");
                    }
                }
                break;
            case "1stSpikeToShoot":
                if(!follower.isBusy()) {
                    shouldIntake = false;
                    if(!hasResetTimer) {
                        shootTimer.reset();
                        hasResetTimer = true;
                    }
                    if(r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() >0.3) {
                        shouldShoot = true;
                        if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                            shouldShoot = false;
                            shouldIntake = true;
                            follower.followPath(shootToCollect2nd);
                            setPathState("collect2ndSpike");
                        }
                    }
                }
                break;
            case "collect2ndSpike":
                shouldIntake = true;
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                        hasResetTimer = false;
                        follower.followPath(collect2ndToShoot);
                        setPathState("2ndSpikeToShoot");
                    }
                }
                break;
            case "2ndSpikeToShoot":
                if(!follower.isBusy()) {
                    shouldIntake = false;
                    if(!hasResetTimer) {
                        shootTimer.reset();
                        hasResetTimer = true;
                    }
                    if(r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() >0.3) {                        shouldShoot = true;
                        if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                            shouldShoot = false;

                            follower.followPath(shootToCollect3rd);
                            setPathState("collect3rdSpike");
                        }
                    }
                }
                break;
            case "collect3rdSpike":
                shouldIntake = true;
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                        hasResetTimer = false;
                        follower.followPath(collect3rdToShoot);
                        setPathState("3rdSpikeToShoot");
                    }
                }
                break;
            case "3rdSpikeToShoot":
                if(!follower.isBusy()) {
                    shouldIntake = false;
                    if(!hasResetTimer) {
                        shootTimer.reset();
                        hasResetTimer = true;
                    }
                    if(r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() >0.3) {                        shouldShoot = true;
                        if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                            subsystemsOn = false;
                            follower.followPath(shootToLeave);
                            setPathState("end");
                        }
                    }
                }
                break;
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
            r.spindexer.update(shouldIntake, shouldShoot, true,false);
        }else{
            r.spindexer.setTargetAngle(240);
        }
    }
    private void turret() {
        if(subsystemsOn) {
            r.turret.updateAutoPower(80);
            r.turret.updatePitch(r.turret.getDistance(follower.getPose()));
            r.turret.updateFlywheelPID();
            r.turret.updateBlackBox(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid());
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