package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Drive {
    public static final float yawMax = 5;
    public static final float yawCheck = 2.5F;
    public static final double MULTIPLIER = 1;
    public static final double STICK_TOLERANCE = 0.1;
    public static final double MINIMUM_TURNING_SPEED = 0.05;
    public static final double ANGULAR_TOLERANCE = Math.toRadians(0.5);
    public static final double x_kP = 0.009;
    public static final double x_kI = 0;
    public static final double x_kD = 0.0001;
    public static final double x_kG = 0;
    public static final int x_maxVel = 5000;
    public static final int x_maxAccel = 500;
    public static final int x_maxJerk = 500;
    public static final double y_kP = 0.006;
    public static final double y_kI = 0;
    public static final double y_kD = 0.001;
    public static final double y_kG = 0;
    public static int y_maxVel = 7000;
    public static int y_maxAccel = 700;
    public static int y_maxJerk = 700;
    public static final int yTarget = 0;
    public static final int xTarget = 500;
}
