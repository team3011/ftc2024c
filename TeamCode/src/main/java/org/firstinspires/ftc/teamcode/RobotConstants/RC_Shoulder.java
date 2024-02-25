package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Shoulder {
    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = -0.25;
    public static int maxVel = 20000;
    public static int maxAccel = 2500;
    public static int maxJerk = 1200;
    public static int dropOffPos = -2100;
    public static int stowPos = 0;
    public static int pickupPos = 0;
    public static int stackAttack = -110;

}
