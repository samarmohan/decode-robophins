package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
public class Limelight {
    private Limelight3A limelight;

    private double llh;
    
    private double llx;
     
    private double lly;

    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
    }

    //runs limelight loop
    public void update(double heading){
        LLResult result = limelight.getLatestResult();

        if (result != null){
            if (result.isValid()){
                Pose3D botPose = result.getBotpose();
                final double METER_TO_INCH = 39.37;
                llh = botPose.getOrientation().getYaw();

                limelight.updateRobotOrientation(llh);
                Pose3D pose_mt2 = result.getBotpose_MT2();
                llx = METER_TO_INCH * pose_mt2.getPosition().x;
                lly = METER_TO_INCH * pose_mt2.getPosition().y;
            }
        }
    }

    public void start(){
        limelight.start();
    }
    public void stop() { limelight.stop(); }
    public double getLlh(){
        return llh;
    }
    
    public double getLlx(){
        return llx;
    }
    
    public double getLly(){
        return lly;
    }
}
