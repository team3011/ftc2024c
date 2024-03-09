package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Drive {
    //public static final float yawMax = 5;
    //public static final float yawCheck = 2.5F;

    public static int red_leftXInverse = -1;
    public static int red_leftYInverse = 1;
    public static int red_rightXInverse = -1;
    public static int blue_leftXInverse = 1;
    public static int blue_leftYInverse = 1;
    public static int blue_rightXInverse = 1;
    public static double rotation_multi = 0.5;                  //decreasing this will reduce how fast we rotate
    public static double autoRotation_multi = 1;
    public static final double MINIMUM_TURNING_SPEED = 0.05;
    public static final double ANGULAR_TOLERANCE = 0.00872665;  //this is 0.5 degrees
    public static double yaw_from_auto = 0;
    public static double x_kP = .035;
    public static double x_kI = 0;
    public static double x_kD = 0.001;
    public static double y_kP = .035;
    public static double y_kI = 0;
    public static double y_kD = 0.002;
    public static double x_velMultiplier = 1;
    public static double y_velMultiplier = 1;
    public static double last_world_linear_accel_y = 0;
    public static double last_world_linear_accel_x = 0;

}
