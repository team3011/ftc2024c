package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;

public class Claw {
    private Servo left;
    private Servo right;
    private TouchSensor sensor;

    public Claw(Servo leftClaw, Servo rightClaw, TouchSensor s) {
        this.left = leftClaw;
        this.right = rightClaw;
        this.sensor = s;
    }
    public void openBottom(){
        this.left.setPosition(RC_Claw.openBottom);
    }

    public void closeBottom(){
        this.left.setPosition(RC_Claw.closeBottom);
    }

    public void openTop(){
        this.right.setPosition(RC_Claw.openTop);
    }

    public void closeTop(){
        this.right.setPosition(RC_Claw.closeTop);
    }

    public void closeTopHard(){
        this.right.setPosition(0.3);
    }

    public void partialBottom(){
        this.left.setPosition(RC_Claw.partialBottom);
    }
    public void partialTop(){
        this.right.setPosition(RC_Claw.partialTop);
    }

    public boolean getClawSensor(){
        return this.sensor.isPressed();
    }
}
