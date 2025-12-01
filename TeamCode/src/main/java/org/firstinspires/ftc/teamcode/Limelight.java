package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
public class Limelight {

    Drivetrain drive = new Drivetrain();
    private Limelight3A limelight;

    private double llx;

    private double lly;

    private double llh;

    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
    }

    //runs limelight loop
    //gets megaTag1&2 data and converts it to bottom left corner as 0,0 coords then sets that in odometry
    public void update(double heading, boolean isLimelightOn){
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();

        if (result != null){
            if (result.isValid()){
                Pose3D botPose = result.getBotpose();
                final double METER_TO_INCH = 39.37;
                llx = METER_TO_INCH * botPose.getPosition().x;
                lly = METER_TO_INCH * botPose.getPosition().y;
                llh = botPose.getOrientation().getYaw();


                if (isLimelightOn){
                    Pose3D pose_mt2 = result.getBotpose_MT2();
                    double mt2x = METER_TO_INCH * pose_mt2.getPosition().x;
                    double mt2y = METER_TO_INCH * pose_mt2.getPosition().y;
                    drive.setOdometryXY(mt2y + 72, (-mt2x) + 72);
                }
            }
        }
    }

    public void start(){
        limelight.start();
    }
    public double getLlx(){
        return llx;
    }
    public double getLly(){
        return lly;
    }
    public double getLlh(){
        return llh;
    }
}
