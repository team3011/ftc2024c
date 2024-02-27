package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Shoulder {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.1;
    public static int maxVel = 400;
    //public static int maxAccel = 2500;
    //public static int maxJerk = 1200;
    public static int dropOffPos = -2100;
    public static int stowPos = 0;
    public static int pickupPos = 0;
    public static int stackAttack = -110;
    public static double ticksFor90 = 14.71;        // 1324 / 90
    public static double startAngle = -16.8;
}
