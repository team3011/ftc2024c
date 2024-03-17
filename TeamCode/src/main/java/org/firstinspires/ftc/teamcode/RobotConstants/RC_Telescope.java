package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Telescope {
    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kF = 0.05;
    public static double velMultiplier = 1.6;
    public static int stowPos = 700;
    public static int pickupPos = 400;
    public static int dropOffHigh = 1000;
    public static int dropOffHighRev = 1000;
    public static int prepForLift = 2200;
}
