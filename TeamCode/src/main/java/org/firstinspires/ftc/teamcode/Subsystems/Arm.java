package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private Claw claw;
    private Lift lift;
    private Wrist wrist;
    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private RevBlinkinLedDriver blinkin;
    private int state = 0;
    private int stage = -3;

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, NavxMicroNavigationSensor n, RevBlinkinLedDriver b, boolean fromAuto){
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
        this.blinkin = b;
    }

    public void moveToPickup(){
        if (this.state != 1) {
            //this.shoulder.setPosition(RC_Shoulder.pickupPos);
            this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
            //this.telescope.setPosition(RobotConstants.telescope_pickupPos);
            //this.claw.openBottom();
            //this.claw.openTop();
            this.state = -1;
            //this.autoPickup = true;
        }
    }

    public void moveToStow() throws InterruptedException {
        if (this.state == -1) {
            //this.claw.closeBottom();
            //Thread.sleep(RobotConstants.claw_pickupPause);
            //this.claw.closeTop();
            //Thread.sleep(RobotConstants.claw_pickupPause);
        }
        if (this.state == 1) {
            //this.claw.partialBottom();
            //Thread.sleep(RobotConstants.claw_dropBottomPause);
            //this.claw.partialTop();
            //Thread.sleep(RobotConstants.claw_dropTopPause);
        }

        //this.shoulder.setPosition(RobotConstants.shoulder_stowPos);
        //this.telescope.setPosition(RobotConstants.telescope_stowPos);
        this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        this.state = 0;
    }

    public void updateEverything() throws InterruptedException {
        this.wrist.update();
        //this.shoulder.update();
        //if (this.autoPickup && this.claw.getClawSensor()) {
        //    moveToStow();
        //    this.autoPickup = false;
        //    this.blinkinLedDriver.setPattern(this.violet);
        //    this.colorTimer.reset();
        //}
        //if (this.colorTimer.seconds()>2){
        //    this.blinkinLedDriver.setPattern(this.mainColor);
        //}
        //this.telescope.update();
    }


}
