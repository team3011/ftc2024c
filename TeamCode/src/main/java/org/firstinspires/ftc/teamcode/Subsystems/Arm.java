package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
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
    private RevBlinkinLedDriver.BlinkinPattern mainColor;
    private RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private int state = 0;
    private int stage = -3;
    private boolean autoPickup = false;
    private ElapsedTime colorTimer = new ElapsedTime();

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, NavxMicroNavigationSensor n, RevBlinkinLedDriver b, boolean fromAuto, boolean isRed){
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
        this.blinkin = b;
        if (isRed){
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }else{
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.blinkin.setPattern(this.mainColor);
    }

    public void manualMoveB(double input){
        //double power = this.shoulder.moveManual(input);
        //this.telescope.moveManual(power/3.33);
        //this.lift.moveManual(input);
        this.telescope.manualMove(input*.5);
    }
    public void moveToPickup(){
        if (this.state != 1) {
            this.shoulder.setPosition(RC_Shoulder.pickupPos);
            this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
            this.telescope.setPosition(RC_Telescope.pickupPos);
            this.claw.openBottom();
            this.claw.openTop();
            this.state = -1;
            this.autoPickup = true;
        }
    }

    public void moveToStow() throws InterruptedException {
        if (this.state == -1) {
            this.claw.closeBottom();
            Thread.sleep(RC_Claw.pickupPause);
            this.claw.closeTop();
            Thread.sleep(RC_Claw.pickupPause);
        }
        if (this.state == 1) {
            this.claw.partialBottom();
            Thread.sleep(RC_Claw.dropBottomPause);
            this.claw.partialTop();
            Thread.sleep(RC_Claw.dropTopPause);
        }

        this.shoulder.setPosition(RC_Shoulder.stowPos);
        this.telescope.setPosition(RC_Telescope.stowPos);
        this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        this.state = 0;
    }

    public void moveToDropOff(){
        if (this.state != -1) {
            this.shoulder.setPosition(RC_Shoulder.dropOffPos);
            this.telescope.setPosition(RC_Telescope.dropOffHigh);
            this.wrist.setTarget(RC_Wrist.dropOffPos, RC_Wrist.dropOffTime);
            this.state = 1;
        }
    }

    public void updateEverything() throws InterruptedException {
        this.wrist.update();
        this.shoulder.update();
        if (this.autoPickup && this.claw.getClawSensor()) {
            moveToStow();
            this.autoPickup = false;
            this.blinkin.setPattern(this.violet);
            this.colorTimer.reset();
        }
        if (this.colorTimer.seconds()>2){
            this.blinkin.setPattern(this.mainColor);
        }
        this.telescope.update();
    }


}
