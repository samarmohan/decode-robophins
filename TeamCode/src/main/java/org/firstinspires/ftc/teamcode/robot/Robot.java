package org.firstinspires.ftc.teamcode.robot;

import android.hardware.lights.Light;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

public class Robot {
    //--- Subsystems ---
    public Drivetrain drivetrain;
    public Intake intake;
    public Lights lights;
    public Spindexer spindexer;
    public Tilt tilt;
    public Turret turret;
    public Limelight limelight;

    List<LynxModule> allHubs;
    //--- Constructor ---
    public Robot(HardwareMap hardwareMap){
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        lights = new Lights(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        tilt = new Tilt(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    //--- Main Loop Functions ---
    public void update() {

    }

    public void clearCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
