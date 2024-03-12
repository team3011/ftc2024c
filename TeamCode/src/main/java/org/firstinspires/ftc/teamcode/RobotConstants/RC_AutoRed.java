package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.Point;

@Config
public class RC_AutoRed {
    public static int x_center_board = -830;
    public static int y_center_board = 470;


    public static int x_center_spike = -1100;
    public static int y_center_spike = 30;

    public static Point center_point = new Point(x_center_board, y_center_board);
    public static Point center_spike_point = new Point(x_center_spike, y_center_spike);

    public static int x_left_board = -845;
    public static int y_left_board = 470;


    public static int x_left_spike = -830;
    public static int y_left_spike = -70;
    public static int x_stack = -1470;
    public static int y_stack = -2050;
    public static Point stack_point = new Point(x_stack, y_stack);
    public static int x_safe = -1450;
    public static int y_safe = 40;
    public static Point point_safe = new Point(x_safe, y_safe);
    public static int cameraLine = 255;
}
