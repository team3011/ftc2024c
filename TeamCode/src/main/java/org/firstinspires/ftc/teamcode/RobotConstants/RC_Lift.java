package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Lift {
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.025;
    public static int maxVel = 5000;
    public static int maxAccel = 500;
    public static int maxJerk = 500;
    public static int maxPos = -3000;
    public static int minPos = -600;
}
