package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public final class Constants {
    //spindexer
    public static double SPINDEXER_GEAR_RATIO = 48.0 / 20.0;
    //odometry
    public static double ODOMETRY_X_OFFSET_MM = -102;
    public static double ODOMETRY_Y_OFFSET_MM = -122;


    // turret PID constants pidf
    @Sorter(sort = 0)
    public static double FLYWHEEL_kP = 0.0005;
    @Sorter(sort = 1)
    public static double FLYWHEEL_kI = 0;
    @Sorter(sort = 2)
    public static double FLYWHEEL_kD = 0;
    @Sorter(sort = 3)
    public static double FLYWHEEL_kF = 0;

    public static double TURRET_kP = 0.01;
    public static double TURRET_kI = 0.0001;
    public static double TURRET_kD = 0.001;

}
