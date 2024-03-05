package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Shoulder {
    public static double kP = 0.06;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kF = 0.1;
    public static double velMultiplier = 1.1;
    public static final double ticksFor90 = 14.71;        // 1324 / 90
    public static final double startAngle = -16.8;
}
