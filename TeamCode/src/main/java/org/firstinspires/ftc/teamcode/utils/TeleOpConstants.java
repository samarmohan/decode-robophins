package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.Sorter;
//
@Configurable()
public final class TeleOpConstants {
    //spindexer
    public static double SPINDEXER_GEAR_RATIO = 48.0 / 20.0;
    //odometry
    public static double ODOMETRY_X_OFFSET_MM = -102;
    public static double ODOMETRY_Y_OFFSET_MM = -122;


    // turret PID constants pidf
    public static double FLYWHEEL_kP = 0.005;
    public static double FLYWHEEL_kI = 0;
    public static double FLYWHEEL_kD = 0;
    public static double FLYWHEEL_kF = 0.72;

    public static double ROTATION_ANGLE_kP = 0.03;
    public static double ROTATION_ANGLE_kI = 0.0;
    public static double ROTATION_ANGLE_kD = 0.001;

    public static double ROTATION_LIMELIGHT_kP = 0.08;
    public static double ROTATION_LIMELIGHT_kI = 0.0;
    public static double ROTATION_LIMELIGHT_kD = 0.0;
    public static double ROTATION_LIMELIGHT_kF = 0.075;
    public static double ROTATION_LIMELIGHT_FF_DEADZONE = 1;


}
