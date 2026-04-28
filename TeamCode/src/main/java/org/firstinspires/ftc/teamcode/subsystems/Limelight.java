package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    // --- Variables ---
    private LLResult result;
    private boolean lastResultWasValid;

    private boolean isTeamRed;

    // --- Hardware ---
    public Limelight3A limelight;
    // --- Constructor ---
    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isTeamRed ? 1 : 0);
        limelight.start();
    }
    // --- Main Loop Function ---
    public void update(){
        LLResult newResult = limelight.getLatestResult();
        if (newResult.isValid()){
            lastResultWasValid = true;
            result = newResult;
        }
        else{
            lastResultWasValid = false;
        }
    }
    // --- Important Functions ---
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    // --- Helpers ---
    public LLResult getResult(){
        return result;
    }
    public boolean wasLastResultValid() {
        return lastResultWasValid;
    }
    public double getTx(){
        if (lastResultWasValid) return result.getTx();
        return 0.0;
    }
    public void setTeam(boolean red){
        isTeamRed = red;
    }
}
