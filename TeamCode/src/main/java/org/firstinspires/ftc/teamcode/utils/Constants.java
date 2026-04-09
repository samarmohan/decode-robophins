package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.Sorter;
//
@Configurable
public final class Constants {
    //spindexer
    public static double SPINDEXER_GEAR_RATIO = 48.0 / 20.0;
    //odometry
    public static double ODOMETRY_X_OFFSET_MM = -102;
    public static double ODOMETRY_Y_OFFSET_MM = -122;


    // turret PID constants pidf
    public static double FLYWHEEL_kP = 0;
    public static double FLYWHEEL_kI = 0;
    public static double FLYWHEEL_kD = 0;
    public static double FLYWHEEL_kF = 0.5;

    public static double TURRET_kP = 0.03;
    public static double TURRET_kI = 0;
    public static double TURRET_kD = 0.001;

}
