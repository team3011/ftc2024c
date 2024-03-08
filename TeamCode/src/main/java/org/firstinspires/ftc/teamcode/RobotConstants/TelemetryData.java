package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TelemetryData {
    public static double wrist_position;
    public static int shoulder_position = 0;
    public static double shoulder_velocity;
    public static int shoulder_target;
    public static double shoulder_power = 0;
    public static double shoulder_pid;
    public static int telescope_position;
    public static double telescope_power;
    public static double telescope_pid;
    public static double telescope_target;
    public static double yaw;
    public static String whatHeadingDo;
    public static int liftStage = -3;
    public static double telescope_current;
    public static double shoulder_current;
}
