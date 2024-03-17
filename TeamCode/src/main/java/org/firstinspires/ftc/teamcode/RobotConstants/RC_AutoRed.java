package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.Point;

@Config
public class RC_AutoRed {
    public static int x_center_board = -830;
    public static int y_center_board = 480;


    public static int x_center_spike = -1120;
    public static int y_center_spike = 30;

    public static Point center_point = new Point(x_center_board, y_center_board);
    public static Point center_spike_point = new Point(x_center_spike, y_center_spike);



    public static int x_left_board = -1050;
    public static int y_left_board = 480;
    public static int x_left_spike = -920;
    public static int y_left_spike = -350;

    public static Point left_point = new Point(x_left_board, y_left_board);
    public static Point left_spike_point = new Point(x_left_spike, y_left_spike);



    public static int x_right_board = -680;
    public static int y_right_board = 480;
    public static int x_right_spike = -930;
    public static int y_right_spike = 180;
    public static Point right_point = new Point(x_right_board, y_right_board);
    public static Point right_spike_point = new Point(x_right_spike, y_right_spike);


    public static int x_stack = -1470;
    public static int y_stack = -2100;
    public static Point stack_point = new Point(x_stack, y_stack);
    public static int x_safe = -1450;
    public static int y_safe = 40;
    public static Point point_safe = new Point(x_safe, y_safe);
    public static int cameraLine = 255;
}
