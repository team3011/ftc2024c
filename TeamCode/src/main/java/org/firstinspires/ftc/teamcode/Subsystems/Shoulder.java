package org.firstinspires.ftc.teamcode.Subsystems;


/*
Instructions on feedforward PID control can be found here:
https://www.youtube.com/watch?v=E6H6Nqe6qJo
 */


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Shoulder {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;
    private boolean resetting = false;

    //private double ticks_in_degrees = [number of ticks]/90.0;
    private double ticks_in_degrees = 1500/90.0;
    private double startAngle = -10;

    /**
     * Class constructor
     * @param m motor obj
     * @param t touch sensor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public Shoulder(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        this.motor.setPower(0);

        if (fromAuto) {
            if (!this.touch.isPressed()) {
                this.resetting = true;
            }
            TelemetryData.shoulder_position = 0;
        }
    }

    /**
     * this will reset the shoulder
     */
    public void resetShoulder(){
        this.resetting = true;
    }

    /**
     * set target position of the shoulder
     * @param t the target position in tick marks,
     *          0 is on the touch sensor
     */
    public void setPosition(int t){
        TelemetryData.shoulder_target = t;
    }

    /**
     * standard update function that will move the shoulder if not at the desired location
     */
    public void update(){
        if (this.resetting) {
            if (this.touch.isPressed()) {
                this.resetting = false;
                TelemetryData.shoulder_position = 0;
            }
        } else {
            this.controller.setPID(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
            TelemetryData.shoulder_position = this.motor.getCurrentPosition();
            double pid = this.controller.calculate(TelemetryData.shoulder_position, TelemetryData.shoulder_target);
            double ff = calcFeedForward();
            double power = pid + ff;
            this.motor.setPower(power);
        }
    }

    /**
     * calculate the feedforward value by taking the cosine of the target angle relative to the ground
     * This works because cos() represents the ratio between the adjacent side of a right triangle and
     * the hypotenuse of the triangle. Whenever the arm is extended straight out (0 degrees),
     * the value of the cosine function is at its maximum (1). This is the point where the system
     * demands the most torque to hold its weight. Whenever we want the arm to be oriented straight
     * up or down (90 degrees), the arm does not require any torque to hold its weight. This matches
     * our feedforward controller, as cos(90 degrees) is 0.
     * By using a nonlinear feedforward controller, we can improve the reliability of our control
     * system and improve the performance of our system.
     *
     * @return the calculated feedforward value
     */
    private double calcFeedForward(){
        double temp = TelemetryData.shoulder_target / this.ticks_in_degrees + this.startAngle;
        return Math.cos(Math.toRadians(temp)) * RC_Shoulder.kF;
    }





}
