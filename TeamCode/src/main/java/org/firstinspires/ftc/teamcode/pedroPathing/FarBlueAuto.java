package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Blue Far Auto", group = "Test")
public class FarBlueAuto extends OpMode {
    private Follower follower;
    private Robot r;
    private Timer pathTimer, opmodeTimer;
    private String pathState;
    //points
    private double INTAKE_SPEED = 0.7;

    // FAR
    private final Pose startPose = new Pose(62, 9, Math.toRadians(180));

    private final Pose shootPose = new Pose(56, 20, Math.toRadians(135));
    private final Pose collectUpWallPose = new Pose(10, 9, Math.toRadians(180));
    private final Pose collectWallRollout = new Pose(10, 25, Math.toRadians(180));
    private final Pose lineUp3rdSpike = new Pose(50, 32, Math.toRadians(180));
    private final Pose collect3rdSpike = new Pose(12, 32, Math.toRadians(180));
    private final Pose leavePose = new Pose(56, 50, Math.toRadians(180));


    //variables
    private boolean shouldIntake = false;
    private boolean shouldShoot = false;
    private boolean subsystemsOn = true;

    //loop functions
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
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
        r.turret.setTeam(false);
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
    private PathChain collect3rdSpikePath, collectGateRunoffPath, collectWallBallsPath;
    private Path wallBallsToShootPath, gateRunoffToShootPath, spikeToShootPath, shootToLeavePath;

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

        collectGateRunoffPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectWallRollout))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        gateRunoffToShootPath = new Path(new BezierLine(collectWallRollout, shootPose));
        gateRunoffToShootPath.setConstantHeadingInterpolation(startPose.getHeading());

        shootToLeavePath = new Path(new BezierLine(shootPose, leavePose));
        shootToLeavePath.setConstantHeadingInterpolation(startPose.getHeading());
    }

    //Finite State Machine
    public void autonomousPathUpdate() {
        switch (pathState) {
            case "shootPreload":
                if (!r.spindexer.isShooting() && shouldShoot) {
                        shouldShoot = false;
                        follower.followPath(collectWallBallsPath, INTAKE_SPEED, true);
                        shouldIntake = true;
                        setPathState("collectWallBalls");
                } else {
                    shouldShoot = true;
                }
                break;
            case "collectWallBalls":
                if(!follower.isBusy()) {
                    follower.followPath(wallBallsToShootPath);
                    shouldIntake = pathTimer.getElapsedTimeSeconds() > 2.0;
                    setPathState("wallBallsToShoot");
                }
                break;
            case "wallBallsToShoot":
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.0) {
                    shouldIntake = false;
                    if (!r.spindexer.isShooting()&& shouldShoot) {
                        shouldShoot = false;
                        shouldIntake = true;
                        follower.followPath(collectGateRunoffPath, INTAKE_SPEED, true);
                        setPathState("collectGateRunoff1");
                    } else {
                        shouldShoot = true;
                    }
                }
                break;
            case "collectGateRunoff1":
                if (!follower.isBusy()) {
                    shouldIntake = true;
                    follower.followPath(gateRunoffToShootPath);
                    setPathState("gateRunoffToShoot1");
                }
                break;
            case "gateRunoffToShoot1":
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.0) {
                    shouldIntake = false;
                    if(!r.spindexer.isShooting()&& shouldShoot) {
                        shouldShoot = false;
                        shouldIntake = true;
                        follower.followPath(collectGateRunoffPath, INTAKE_SPEED, true);
                        setPathState("collectGateRunoff2");
                    }else{
                        shouldShoot = true;
                    }
                }
                break;
            case "collectGateRunoff2":
                if (!follower.isBusy()) {
                    shouldIntake = true;
                    follower.followPath(gateRunoffToShootPath);
                    setPathState("gateRunoffToShoot2");
                }
                break;
            case "gateRunoffToShoot2":
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.0) {
                    shouldIntake = false;
                    if(!r.spindexer.isShooting() && shouldShoot) {
                        shouldShoot = false;
                        shouldIntake = true;
                        follower.followPath(collect3rdSpikePath, INTAKE_SPEED, true);
                        setPathState("collect3rdSpike");
                    } else {
                        shouldShoot = true;
                    }
                }
                break;
            case "collect3rdSpike":
                if (!follower.isBusy()) {
                    shouldIntake = true;
                    follower.followPath(spikeToShootPath);
                    setPathState("shootSpike");
                }
                break;
            case "shootSpike":
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.0) {
                    shouldIntake = false;
                    if(!r.spindexer.isShooting() && shouldShoot) {
                        shouldShoot = false;
                        shouldIntake = false;

                        follower.followPath(shootToLeavePath);
                        setPathState("end");
                        subsystemsOn = false;
                    }else{
                        shouldShoot = true;
                    }
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
            r.spindexer.update(shouldIntake, shouldShoot, true,false);
        }else{
            r.spindexer.setTargetAngle(240);
        }
    }
    private void turret() {
        if(subsystemsOn) {
            r.turret.updateAutoPower(r.turret.getDistance(follower.getPose()));
            r.turret.updatePitch(r.turret.getDistance(follower.getPose()));
            r.turret.updateFlywheelPID();
            r.turret.updateBlackBox(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()), r.limelight.getTx(), r.limelight.wasLastResultValid());
        }
        else{
            r.turret.updateAutoPower(0);
            r.turret.aimForward();
        }
    }
    public void telemetry(){
        // Feedback to Driver Hub for debugging
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
        telemetry.update();
    }
}
