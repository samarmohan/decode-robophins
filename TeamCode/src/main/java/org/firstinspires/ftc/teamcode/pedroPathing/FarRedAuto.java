package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.utils.AutonPoseSave;

@Autonomous(name = "Red Far Auto", group = "Test")
public class FarRedAuto extends OpMode {
    private Follower follower;
    private Robot r;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Timer pathTimer, opmodeTimer;
    private ElapsedTime shootTimer;
    private String pathState;
    //points
    private double INTAKE_SPEED = 0.7;

    // FAR
    private final Pose startPose = new Pose(144 - 54, 9, Math.toRadians(180 - 180));

    private final Pose shootPose = new Pose(144 - 56, 20, Math.toRadians(180 - 180));
    private final Pose collectUpWallPose = new Pose(144 - 10, 9, Math.toRadians(180 - 180));
    private final Pose lineUp3rdSpike = new Pose(144 - 50, 32, Math.toRadians(180 - 180));
    private final Pose collect3rdSpike = new Pose(144 - 12, 32, Math.toRadians(180 - 180));
    private final Pose leavePose = new Pose(144 - 56, 50, Math.toRadians(180 - 180));


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
        setPathState("shootPreload");
        r.turret.setTeam(true);
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

    //Pathing
    private PathChain collect3rdSpikePath, collectWallBallsPath;
    private Path wallBallsToShootPath, spikeToShootPath, shootToLeavePath;

    public void buildPaths(){
        collectWallBallsPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, collectUpWallPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        wallBallsToShootPath = new Path(new BezierLine(collectUpWallPose, shootPose));
        wallBallsToShootPath.setConstantHeadingInterpolation(startPose.getHeading());

        collect3rdSpikePath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lineUp3rdSpike))
                .setConstantHeadingInterpolation(startPose.getHeading())

                .addPath(new BezierLine(lineUp3rdSpike, collect3rdSpike))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        spikeToShootPath = new Path(new BezierLine(collect3rdSpike, shootPose));
        spikeToShootPath.setConstantHeadingInterpolation(startPose.getHeading());

        shootToLeavePath = new Path(new BezierLine(shootPose, leavePose));
        shootToLeavePath.setConstantHeadingInterpolation(startPose.getHeading());
    }

    //Finite State Machine
    public void autonomousPathUpdate() {
        if(opmodeTimer.getElapsedTimeSeconds() < 28){
            switch (pathState) {
                case "shootPreload":
                    shouldShoot = true;
                    if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                        shouldShoot = false;
                        follower.followPath(collectWallBallsPath);
                        setPathState("collectWallBalls");
                    }
                    break;
                case "collectWallBalls":
                    shouldIntake = true;
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                            hasResetTimer = false;
                            follower.followPath(wallBallsToShootPath);
                            setPathState("wallBallsToShoot");
                        }
                    }
                    break;
                case "wallBallsToShoot":
                    if (!follower.isBusy()) {
                        shouldIntake = false;
                        if (!hasResetTimer) {
                            shootTimer.reset();
                            hasResetTimer = true;
                        }
                        if (r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() > 0.3) {
                            shouldShoot = true;
                            if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                                shouldShoot = false;
                                shouldIntake = true;
                                follower.followPath(collect3rdSpikePath);
                                setPathState("collect3rdSpike");
                            }
                        }
                    }
                    break;
                case "collect3rdSpike":
                    shouldIntake = true;
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                            hasResetTimer = false;
                            follower.followPath(spikeToShootPath);
                            setPathState("shootSpike");
                        }
                    }
                    break;
                case "shootSpike":
                    if (!follower.isBusy()) {
                        shouldIntake = false;
                        if (!hasResetTimer) {
                            shootTimer.reset();
                            hasResetTimer = true;
                        }
                        if (r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() > 0.3) {
                            shouldShoot = true;
                            if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                                shouldShoot = false;

                                follower.followPath(collectWallBallsPath);
                                setPathState("collectGateRunoff1");
                            }
                        }
                    }
                    break;
                case "collectGateRunoff1":
                    shouldIntake = true;
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                            hasResetTimer = false;
                            follower.followPath(wallBallsToShootPath);
                            setPathState("gateRunoffToShoot1");
                        }
                    }
                    break;
                case "gateRunoffToShoot1":
                    if (!follower.isBusy()) {
                        shouldIntake = false;
                        if (!hasResetTimer) {
                            shootTimer.reset();
                            hasResetTimer = true;
                        }
                        if (r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() > 0.3) {
                            shouldShoot = true;
                            if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                                shouldShoot = false;
                                follower.followPath(collectWallBallsPath);
                                setPathState("collectGateRunoff2");
                            }
                        }
                    }
                    break;
                case "collectGateRunoff2":
                    shouldIntake = true;
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                            hasResetTimer = false;
                            follower.followPath(wallBallsToShootPath);
                            setPathState("gateRunoffToShoot2");
                        }
                    }
                    break;
                case "gateRunoffToShoot2":
                    if (!follower.isBusy()) {
                        shouldIntake = false;
                        if (!hasResetTimer) {
                            shootTimer.reset();
                            hasResetTimer = true;
                        }
                        if (r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() > 0.3) {
                            shouldShoot = true;
                            if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                                shouldShoot = false;
                                follower.followPath(collectWallBallsPath);
                                setPathState("collectGateRunoff3");
                            }
                        }
                    }
                    break;
                case "collectGateRunoff3":
                    shouldIntake = true;
                    if (!follower.isBusy()) {
                        if (pathTimer.getElapsedTimeSeconds() > 2 || r.spindexer.isFull()) {
                            hasResetTimer = false;
                            follower.followPath(wallBallsToShootPath);
                            setPathState("gateRunoffToShoot3");
                        }
                    }
                    break;
                case "gateRunoffToShoot3":
                    if (!follower.isBusy()) {
                        shouldIntake = false;
                        if (!hasResetTimer) {
                            shootTimer.reset();
                            hasResetTimer = true;
                        }
                        if (r.spindexer.getState() == Spindexer.SpindexerState.READY_TO_SHOOT && shootTimer.seconds() > 0.3) {
                            shouldShoot = true;
                            if (!r.spindexer.isShooting() && !r.spindexer.hasBalls()) {
                                subsystemsOn = false;
                                follower.followPath(shootToLeavePath);
                                setPathState("end");
                            }
                        }
                        break;
                    }
                case "end":
                    AutonPoseSave.lastAutonPose = follower.getPose();
                    break;
            }
        }
        else{
            subsystemsOn = false;
            AutonPoseSave.lastAutonPose = follower.getPose();
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
