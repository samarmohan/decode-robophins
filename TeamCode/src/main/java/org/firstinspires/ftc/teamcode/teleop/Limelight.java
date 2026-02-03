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

    private double tx;

    private double ty;

    private double ta;
    private boolean valid;

    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
    }

    //runs limelight loop
    public void update(double heading){
        LLResult result = limelight.getLatestResult();
        valid = false;

        if (result != null){
            if (result.isValid()){
                valid = true;
                //Pose3D botPose = result.getBotpose();
                final double METER_TO_INCH = 39.37;
                /*
                llh = botPose.getOrientation().getYaw();

                limelight.updateRobotOrientation(llh);
                Pose3D pose_mt2 = result.getBotpose_MT2();
                llx = METER_TO_INCH * pose_mt2.getPosition().x;
                lly = METER_TO_INCH * pose_mt2.getPosition().y;

                 */

                tx = result.getTx();
                ty = result.getTy();
                ta = result.getTa();
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

    public double getTx(){
        return tx;
    }

    public double getTa() {
        return ta;
    }

    public double getTy() {
        return ty;
    }

    public boolean isResultValid(){
        return valid;
    }
}
