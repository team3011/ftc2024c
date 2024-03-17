package org.firstinspires.ftc.teamcode.Subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private Claw claw;
    private Lift lift;
    private Wrist wrist;
    private AHRS navx;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern mainColor;
    private RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private int state = 0;
    private boolean autoPickup = false;
    private ElapsedTime colorTimer = new ElapsedTime();
    private boolean isRedHere;

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, AHRS n, RevBlinkinLedDriver b, boolean fromAuto, boolean isRed){
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
        this.blinkin = b;
        this.isRedHere = isRed;
        if (isRed){
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }else{
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.blinkin.setPattern(this.mainColor);
    }

    public void reset() {
        this.shoulder.reset();
        this.telescope.reset();
    }

    public double getPitch(){
        //Orientation angles = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double robotPitch = Math.toRadians(angles.secondAngle);
        //return robotPitch;
        return 0;
    }

    public void lifting(double input){
        input = input *.5;
        double pitch = getPitch();
        if (Math.abs(input)>0 && pitch<-.01) {
            double modifier = input + (pitch/-.07)*.5;
            this.lift.moveManual(modifier);
            if (TelemetryData.telescope_position < 350){
                this.telescope.manualMove(0);
            }
        } else if (Math.abs(input)>0 && pitch > .01) {
            double modifier = input - (pitch/.07)*.5;
            this.lift.moveManual(modifier);
            if (TelemetryData.telescope_position < 350){
                this.telescope.manualMove(0);
            }
        } else {
            this.lift.moveManual(input);
            if (TelemetryData.telescope_position < 350){
                this.telescope.manualMove(0);
            } else {
                this.telescope.manualMove(-input);
            }
        }
    }

    public void manualMoveB(double input, boolean lift){
        this.telescope.manualMove(input*.5);
        this.telescope.getCurrent();
        if (lift && TelemetryData.telescope_current > 700) {
            double modifier = 1 - (900 - TelemetryData.telescope_current) / 200.;
            this.lift.moveManual(-modifier);
        }
    }

    public void manualMoveA(double input, boolean lift){
        this.shoulder.moveManual(input*.5);
        if (input != 0) {
            if (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5) {
                double revWrist = 0.2;
                if (lift && TelemetryData.shoulder_position > 500) {
                    revWrist = revWrist - (TelemetryData.shoulder_position - 500) / 3000.0;
                }
                if (revWrist > 0.63) {
                    revWrist = 0.25;
                } else if (revWrist < 0.07) {
                    revWrist = 0.07;
                }
                RC_Wrist.dropOffPosRev = revWrist;
                this.wrist.setPosition(revWrist);

                //@500 shoulder i need .2 wrist
                //@800 shoulder i need .1 wrist

                //RC_Wrist.dropOffPosRev = revWrist;
                //this.wrist.setTarget(revWrist,1);
                //TelemetryData.wrist_position = revWrist;
                //this.wrist.update();
            }

            this.shoulder.getCurrent();
            if (TelemetryData.shoulder_current > 2250) {
                double modifier = 1 - (2550 - TelemetryData.shoulder_current) / 300. + .1;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        }
    }

    public void stopLift(){
        this.lift.moveManual(0);
    }
    public void moveToPickup(boolean dropping){
        if (this.state != 1) {
            if (!dropping) {
                this.claw.openBottom();
                this.claw.openTop();
                this.autoPickup = true;
                this.shoulder.setPosition(RC_Shoulder.pickupPos);
                TelemetryData.shoulder_target = RC_Shoulder.pickupPos;
            } else {
                this.shoulder.setPosition(RC_Shoulder.pickupPos + 100);
                TelemetryData.shoulder_target = RC_Shoulder.pickupPos + 100;
            }

            this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
            this.telescope.setPosition(RC_Telescope.pickupPos);
            this.state = -1;
        }
    }

    public void autoMoveToPickup(){
        this.shoulder.setPosition(RC_Shoulder.pickupPos + 100);
        TelemetryData.shoulder_target = RC_Shoulder.pickupPos + 100;
        this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
        this.telescope.setPosition(RC_Telescope.pickupPos);

    }

    public void moveToPickupTruchon(boolean dropping){
        if (this.state != 1) {
            if (!dropping) {
                this.claw.openBottom();
                this.claw.openTop();
                this.autoPickup = true;
                this.shoulder.setPosition(RC_Shoulder.pickupPos);
                TelemetryData.shoulder_target = RC_Shoulder.pickupPos;
            } else {
                this.shoulder.setPosition(RC_Shoulder.pickupPos + 100);
                TelemetryData.shoulder_target = RC_Shoulder.pickupPos + 100;
            }

            this.wrist.setTarget(RC_Wrist.pickupPos, 100);
            this.telescope.setPosition(RC_Telescope.pickupPos);
            this.state = -1;
        }
    }

    public void prepStackAttack(){
        this.claw.openBottom();
        this.claw.openTop();
        this.autoPickup = true;
        this.shoulder.setPosition(RC_Shoulder.pickupPos + 250);
        TelemetryData.shoulder_target = RC_Shoulder.pickupPos + 250;
        this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
        this.telescope.setPosition(RC_Telescope.pickupPos);
        this.state = -1;
    }

    public void stackAttack(){
        this.autoPickup = true;
        this.shoulder.setPosition(RC_Shoulder.pickupPos);
        TelemetryData.shoulder_target = RC_Shoulder.pickupPos;
    }

    /**
     *
     * @param p pixels to drop
     * @throws InterruptedException
     */
    public void moveToStow(int p) throws InterruptedException {
        if (p == 2) {
            if (this.state == -1) {
                this.claw.closeBottom();
                Thread.sleep(RC_Claw.pickupPause);
                this.claw.closeTop();
                Thread.sleep(RC_Claw.pickupPause);
                this.shoulder.setPosition(RC_Shoulder.stowPos);
            }
            if (this.state == 1) {
                this.claw.partialBottom();
                Thread.sleep(RC_Claw.dropBottomPause);
                this.claw.partialTop();
                Thread.sleep(RC_Claw.dropTopPause);
                this.shoulder.setPosition(RC_Shoulder.stowPos);
            }
        } else if (p == 1){
            if (this.state == 1) {
                this.claw.partialBottom();
                Thread.sleep(RC_Claw.dropBottomPause);
                this.shoulder.setPosition(RC_Shoulder.stowPos + 100);
            }
        } else if (p == 0){
            this.claw.openBottom();
            this.claw.openTop();
            Thread.sleep(RC_Claw.dropBottomPause);
            this.shoulder.setPosition(RC_Shoulder.stowPos);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        }
        this.telescope.setPosition(RC_Telescope.stowPos);
        this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        this.state = 0;
    }

    public void autoStow() throws InterruptedException {
        this.claw.partialBottom();
        Thread.sleep(RC_Claw.dropBottomPause);
        this.shoulder.setPosition(RC_Shoulder.stowPos);
        this.telescope.setPosition(RC_Telescope.stowPos + 300);
        this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);

    }

    public void moveToDropOff(){
        if (this.state != -1) {
            if (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5){
                //we are facing the board
                this.shoulder.setPosition(RC_Shoulder.dropOffPosRev);
                this.telescope.setPosition(RC_Telescope.dropOffHighRev);
                this.wrist.setTarget(RC_Wrist.dropOffPosRev, RC_Wrist.dropOffTime);
            } else {
                //we are facing normally
                this.shoulder.setPosition(RC_Shoulder.dropOffPos);
                this.telescope.setPosition(RC_Telescope.dropOffHigh);
                this.wrist.setTarget(RC_Wrist.dropOffPos, RC_Wrist.dropOffTime);
            }
            this.state = 1;
        }
    }

    public void autoMoveToDropOff(){
                this.shoulder.setPosition(RC_Shoulder.dropOffPos);
                this.telescope.setPosition(RC_Telescope.dropOffHigh);
                this.wrist.setTarget(RC_Wrist.dropOffPos, RC_Wrist.dropOffTime);
            this.state = 1;

    }

    public void updateEverything() throws InterruptedException {
        if (this.state == 0 && Math.abs(TelemetryData.telescope_target - TelemetryData.telescope_position) > 50 &&
                (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5)) {
            //we need to move the telescope first
            this.telescope.update();
        } else {
            this.wrist.update();
            this.shoulder.update();
            if (this.autoPickup && this.claw.getClawSensor()) {
                moveToStow(2);
                this.autoPickup = false;
                this.blinkin.setPattern(this.violet);
                this.colorTimer.reset();
            }
            if (this.colorTimer.seconds() > 2) {
                this.blinkin.setPattern(this.mainColor);
            }
            this.telescope.update();
        }
    }

    public void prepForLift(double input) throws InterruptedException {
        input = input * .6;
        if (TelemetryData.liftStage == -3){
            //move up to clear the cord and retract telescope
            TelemetryData.shoulder_target = 200;
            TelemetryData.telescope_target = 0;
            this.shoulder.update();
            this.telescope.update();
            if (this.telescope.returnStage() == 0 & Math.abs(TelemetryData.shoulder_target - TelemetryData.shoulder_position) < 100) {
                TelemetryData.liftStage = -2;
            }
        } else if (TelemetryData.liftStage == -2) {
            //move down to engage the cord
            TelemetryData.shoulder_target = 0;
            TelemetryData.telescope_target = 0;
            this.shoulder.update();
            this.telescope.update();
            if (this.telescope.returnStage() < 1 & Math.abs(TelemetryData.shoulder_target - TelemetryData.shoulder_position) < 100) {
                TelemetryData.liftStage = -10;
            }
        } else if (TelemetryData.liftStage == -1) {
            //push telescope out to engage cable
            if (TelemetryData.telescope_position < 300) {
                this.shoulder.update();
                this.telescope.manualMove(-input);
                this.telescope.getCurrent();
                if (TelemetryData.telescope_current>600){
                    double modifier = 1-(800-TelemetryData.telescope_current)/200.;
                    this.lift.moveManual(-modifier);
                } else {
                    this.lift.moveManual(0);
                }
            } else {
                TelemetryData.liftStage = 0;
                this.telescope.manualMove(0);
                this.shoulder.update();
                this.lift.moveManual(0);
            }
        } else if (TelemetryData.liftStage == 0) {
            //move shoulder into position to lift
            this.telescope.manualMove(0);
            if (TelemetryData.telescope_position < 300) {
                this.shoulder.update();
                this.telescope.manualMove(-input);
            } else if (TelemetryData.telescope_position > 400) {
                this.shoulder.update();
                this.telescope.manualMove(input * .2);
            }
            if (TelemetryData.shoulder_position < 1400) {
                double speed = this.shoulder.moveManual(-input);
                this.shoulder.getCurrent();
                //this.telescope.manualMove(speed / 3.33);
                this.telescope.manualMove(0);
                if (TelemetryData.shoulder_current > 2250) {
                    double modifier = 1 - (2550 - TelemetryData.shoulder_current) / 300. + .1;
                    this.lift.moveManual(-modifier);
                } else {
                    this.lift.moveManual(0);
                }
            } else {
                TelemetryData.liftStage = 1;
                this.telescope.manualMove(0);
                this.shoulder.moveManual(0);
                this.lift.moveManual(0);
            }
        } else if (TelemetryData.liftStage == 1) {
            //move telescope up to above bar
            this.shoulder.updatePosition();
            this.telescope.updatePosition();
            this.shoulder.moveManual(0);
            if (TelemetryData.telescope_position < RC_Telescope.prepForLift) {
                if (TelemetryData.shoulder_position < 1400) {
                    this.shoulder.moveManual(-input);
                } else if (TelemetryData.shoulder_position > 1500){
                    this.shoulder.moveManual(input*.2);
                } else {
                    this.shoulder.moveManual(0);
                }


                this.telescope.manualMove(-input);
                this.telescope.getCurrent();
                if (TelemetryData.telescope_current > 600) {
                    double modifier = 1 - (800 - TelemetryData.telescope_current) / 200.;
                    this.lift.moveManual(-modifier);
                }
            } else {
                TelemetryData.liftStage = 2;
                this.telescope.manualMove(0);
                this.shoulder.moveManual(0);
                this.lift.moveManual(0);
            }
        }else if (TelemetryData.liftStage == 2) {
            this.telescope.setPosition(RC_Telescope.prepForLift);
            this.shoulder.setPosition(1500);
            //updateEverything();
            //this.lift.moveManual(0);
        }
            /*
            this.shoulder.updatePosition();
            this.telescope.updatePosition();
            if (TelemetryData.shoulder_position < 1400) {
                this.shoulder.moveManual(-input);
            } else if (TelemetryData.shoulder_position > 1500){
                this.shoulder.moveManual(input*.2);
            } else {
                this.shoulder.moveManual(0);
            }
            //move telescope down to give slack in line
            if (TelemetryData.telescope_position > RC_Telescope.prepForLift - 300) {
                this.telescope.manualMove(input);
                if (TelemetryData.shoulder_position < 1400) {
                    this.shoulder.moveManual(-input);
                } else if (TelemetryData.shoulder_position > 1500){
                    this.shoulder.moveManual(input*.2);
                } else {
                    this.shoulder.moveManual(0);
                }
            } else {
                TelemetryData.liftStage = 3;
            }
        } else if (TelemetryData.liftStage == 3) {
            this.telescope.setPosition(RC_Telescope.prepForLift - 300);
            this.shoulder.setPosition(1400);
            updateEverything();
            this.lift.moveManual(0);
        }

             */
    }

    public void initialMove(){
        this.shoulder.setPosition(RC_Shoulder.stowPos+150);
        this.telescope.setPosition(0);
        this.claw.closeTop();
        this.claw.closeBottom();
    }
}
