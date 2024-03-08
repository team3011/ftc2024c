package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Wrist {
    public Servo leftWrist;
    public Servo rightWrist;
    private double target;
    private double currentPos;
    private int time;
    private ElapsedTime timer = new ElapsedTime();
    private double moveIncrement;

    /**
     * Class constructor
     *
     * @param leftJoint servo obj
     * @param rightJoint servo obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public Wrist(Servo leftJoint, Servo rightJoint, boolean fromAuto) {
        this.leftWrist = leftJoint;
        this.rightWrist = rightJoint;
        this.rightWrist.setDirection(Servo.Direction.REVERSE);

        if (fromAuto) {
            this.leftWrist.setPosition(RC_Wrist.stowPos);
            this.rightWrist.setPosition(RC_Wrist.stowPos);
            TelemetryData.wrist_position = RC_Wrist.stowPos;
            this.target = RC_Wrist.stowPos;
        } else {
            this.leftWrist.setPosition(TelemetryData.wrist_position);
            this.rightWrist.setPosition(TelemetryData.wrist_position);
            this.target = TelemetryData.wrist_position;
        }
    }

    public void setPosition(double input){
        this.leftWrist.setPosition(input);
        this.rightWrist.setPosition(input);
        this.currentPos = input;
        this.target = input;
        TelemetryData.wrist_position = this.currentPos;
    }

    /**
     * Will set desired target for the servo and the time it will take to get there
     * servo will move gradually to target in the update function
     *
     * @param input value between 0 and 1, make sure servo can handle it
     * @param t time it will take to go from current location to new location in milliseconds,
     *          a value of 0 will move at the max servo speed
     */
    public void setTarget(double input, int t) {
        if (input != this.currentPos) {
            this.target = input;
            this.time = t;
            this.timer.reset();
            this.moveIncrement = (input - this.currentPos) / t;
        }
    }

    /**
     * standard update function that will move the wrist if not at the desired location
     */
    public void update(){
        double duration = this.timer.milliseconds();
        if (Math.abs(this.currentPos-this.target)>.01 && duration < this.time) {
            double temp = this.currentPos + duration * this.moveIncrement;
            this.leftWrist.setPosition(temp);
            this.rightWrist.setPosition(temp);
            //return temp;
        } else {
            this.leftWrist.setPosition(this.target);
            this.rightWrist.setPosition(this.target);
            this.currentPos = this.target;
        }
        TelemetryData.wrist_position = this.currentPos;
    }
}
