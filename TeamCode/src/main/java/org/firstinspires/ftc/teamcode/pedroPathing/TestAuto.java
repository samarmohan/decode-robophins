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

@Autonomous(name = "Test Auto", group = "Test")
public class TestAuto extends OpMode{
    private Follower follower;
    private Robot r;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    //points
    private final Pose startPose = new Pose(0,0,Math.toRadians(0));
    private final Pose endPose = new Pose(15, 0, Math.toRadians(90));

    //variables
    private boolean intake = false;
    private boolean shoot = false;

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
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        r.clearCache();
        autonomousPathUpdate();
        intake();
        spindexer();
        turret();
        telemetry();
    }

    @Override
    public void stop() {}

    //Pathing
    private Path moveForward;

    public void buildPaths(){
        moveForward = new Path(new BezierLine(startPose, endPose));
        moveForward.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    //Finite State Machine
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(moveForward);
                intake = true;
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 10){
                    shoot = true;
                    intake = false;
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    //robot subsystems
    private void intake(){
        if(intake){
            r.intake.intake();
        }else{
            r.intake.outtake();
        }
        r.intake.run();
    }
    private void spindexer(){
        r.spindexer.update(intake, shoot, true,false);
    }
    private void turret(){
        r.turret.updateAutoPower(220);
        r.turret.updatePitch(220);
        r.turret.updateFlywheelPID();
        r.turret.updatePositionAim(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()));
    }
    public void telemetry(){
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Target Flywheel RPM", r.turret.getTargetRPM());
        telemetry.addData("Actual Flywheel RPM", r.turret.getFlywheelRPM());
        telemetry.addData("Flywheel Power", r.turret.getFlywheelPower());
        telemetry.addLine("------------------------------------");
        telemetry.addData("Turret Angle", r.turret.getTurretAngle());
        telemetry.addData("Turret Target Angle", r.turret.getTargetAngle());
        telemetry.addData("Turret Power", r.turret.getTurretPower());
        telemetry.addData("is running Bang Bang", r.turret.getBangBang());
        telemetry.addData("Limelight Valid", r.limelight.wasLastResultValid());
        telemetry.addData("Target Pitch", r.turret.getPitch());
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
