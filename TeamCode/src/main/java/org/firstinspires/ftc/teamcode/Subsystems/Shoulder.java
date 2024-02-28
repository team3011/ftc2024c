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
    private boolean resetTriggered = false;
    private double lastPIDCalc = .001;

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
            } else {
                TelemetryData.shoulder_position = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
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
        if (!this.resetTriggered && this.touch.isPressed()) {
            TelemetryData.shoulder_position = 0;
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.resetTriggered = true;
        } else if (!this.touch.isPressed()){
            this.resetTriggered = false;
        }
        this.controller.setPID(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
        double pid = this.controller.calculate(TelemetryData.shoulder_position, TelemetryData.shoulder_target);
        pid = pid/10;

        TelemetryData.shoulder_pid = pid;
        //this will limit the pid to a range of -1 to 1
        if (pid > 1) {
            pid = 1;
        } else if (pid < -1) {
            pid = -1;
        }

        //***********************************************
        //this section reduces the acceleration by reducing how fast the pid calculation can change
        if (lastPIDCalc > 0) {
            if (pid > 0) {
                if (pid - lastPIDCalc > .2) {
                    pid = lastPIDCalc * RC_Shoulder.velMultiplier;
                }
            } else {
                pid = -0.001;
            }
        } else {
            if (pid < 0) {
                if (pid - lastPIDCalc < -0.2) {
                    pid = lastPIDCalc * RC_Shoulder.velMultiplier;
                }

            } else {
                pid = 0.001;
            }
        }
        lastPIDCalc = pid;
        //**********************************************

        double ff = calcFeedForward();
        double power = pid + ff;

        if (power > 1) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }

        if (this.touch.isPressed() && power < 0){
            power = 0;
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TelemetryData.shoulder_position = 0;
        } else {
            //limits the speed right before it touches the switch
            if (power < 0 && TelemetryData.shoulder_position < 500) {
                if (power < -0.1) {
                    power = -0.1;
                } else {
                    power = -0.1;
                }
            }
            //limits the speed after straight up
            if (power > 0 && TelemetryData.shoulder_position > 1600) {
                if (power > 0.1) {
                    power = 0.1;
                }
            }

        }
        this.motor.setPower(power);
        TelemetryData.shoulder_velocity = this.motor.getVelocity();
        TelemetryData.shoulder_power = power;
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
        double temp = TelemetryData.shoulder_position / RC_Shoulder.ticksFor90 + RC_Shoulder.startAngle;
        return Math.cos(Math.toRadians(temp)) * RC_Shoulder.kF;
    }
}
