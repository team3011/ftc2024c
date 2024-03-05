package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private Claw claw;
    private Lift lift;
    private Wrist wrist;
    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private int state = 0;
    private int stage = -3;

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, NavxMicroNavigationSensor n){
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
    }
}
