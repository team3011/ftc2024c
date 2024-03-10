package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Telescope {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;
    private boolean resetting = false;
    private int resetStage = 0;
    private boolean resetTriggered = false;
    private double lastPIDCalc = .001;
    private int lastTarget = 0;
    private boolean manualEngaged = false;
    private boolean justTouched = false;

    /**
     * Class constructor
     * @param m motor obj
     * @param t touch sensor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public Telescope(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        this.motor.setPower(0);

        if (fromAuto) {
            if (!this.touch.isPressed()) {
                this.resetStage = 1;
                this.justTouched = true;
            } else {
                TelemetryData.shoulder_position = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.resetStage = 0;
            }
        }
    }

    public void reset(){
        TelemetryData.telescope_position = 0;
        TelemetryData.telescope_target = 0;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void manualMove(double input) {
        if (input != 0) {
            this.manualEngaged = true;
            TelemetryData.telescope_position = this.motor.getCurrentPosition();
            TelemetryData.telescope_target = TelemetryData.telescope_position;
            if (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5) {
                RC_Telescope.dropOffHighRev = TelemetryData.telescope_position;
            } else {
                RC_Telescope.dropOffHigh = TelemetryData.telescope_position;
            }
            if (this.touch.isPressed() && input < 0) {
                this.motor.setPower(0);
                this.justTouched = true;
                this.resetTriggered = true;
                TelemetryData.telescope_power = 0;
                TelemetryData.telescope_position = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (TelemetryData.telescope_position > 2800 && input > 0) {
                this.motor.setPower(0);
                TelemetryData.telescope_power = 0;
            } else {
                this.motor.setPower(input);
                TelemetryData.telescope_power = input;
                this.resetTriggered = false;
            }
        } else {
            this.manualEngaged = false;
            //if (this.touch.isPressed()) {
            //    this.motor.setPower(.2);
            //} else {
            //    this.motor.setPower(0);
           // }
        }

    }

    public void setPosition(int t){
        TelemetryData.telescope_target = t;
    }

    /**
     * standard update function that will move the shoulder if not at the desired location
     */
    public void update(){
        if (!this.manualEngaged) {
            double power = 0;
            TelemetryData.telescope_position = this.motor.getCurrentPosition();
            //our target is either 0 or not 0
            if (TelemetryData.telescope_target == 0) {
                //target is 0, if we are far away set reset to true
                if (TelemetryData.shoulder_target == 0 && TelemetryData.shoulder_position > 50) {
                    this.resetStage = 1;
                }
                //we need to touch and back off the sensor
                if (this.touch.isPressed()) {
                    power = 0.1;
                    this.resetTriggered = true;
                    this.resetStage = -1;
                    this.justTouched = true;

                } else {
                    if (this.resetStage == 1) {
                        //heading towards sensor, slow down if close else full speed
                        if (TelemetryData.telescope_position > 500) {
                            power = calcPower();
                        } else {
                            power = -.2;
                        }
                    } else if (this.resetStage == -1) {
                        //we just touched the sensor and this is the first time it was not touched, so set this point as 0
                        power = 0;
                        this.resetStage = 0;
                        TelemetryData.telescope_position = 0;
                        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            } else {
                this.resetTriggered = false;
                this.resetStage = 1;
                //this is in case of an emergency
                if (this.touch.isPressed()) {
                    power = 0.3;
                } else if (TelemetryData.shoulder_target == RC_Shoulder.dropOffPos) {
                    //do NOT move the telescope until the shoulder is in position
                    //there is too much rotational inertia and it will skip the shoulder
                    if (TelemetryData.shoulder_position > 1300 ) {
                        power = calcPower();
                    } else {
                        power = 0;
                    }
                } else if (Math.abs(TelemetryData.telescope_position - TelemetryData.telescope_target) > 10) {
                    power = calcPower();
                } else {
                    power = 0;
                }
            }
            this.motor.setPower(power);
            TelemetryData.telescope_power = power;
        }
    }

    public int returnStage(){
        return this.resetStage;
    }

    private double calcPower() {
        double power = 0;
        this.controller.setPID(RC_Telescope.kP, RC_Telescope.kI, RC_Telescope.kD);
        double pid = this.controller.calculate(TelemetryData.telescope_position, TelemetryData.telescope_target);
        pid = pid/10;

        TelemetryData.telescope_pid = pid;
        //this will limit the pid to a range of -1 to 1
        if (pid > 1) {
            pid = 1;
        } else if (pid < -1) {
            pid = -1;
        }

        //***********************************************
        //this section reduces the acceleration by reducing how fast the pid calculation can change
        if (this.lastPIDCalc > 0) {
            if (pid > 0) {
                if (pid - this.lastPIDCalc > .2) {
                    pid = this.lastPIDCalc * RC_Telescope.velMultiplier;
                }
            } else {
                pid = -0.001;
            }
        } else {
            if (pid < 0) {
                if (pid - this.lastPIDCalc < -0.2) {
                    pid = this.lastPIDCalc * RC_Telescope.velMultiplier;
                }

            } else {
                pid = 0.001;
            }
        }
        this.lastPIDCalc = pid;
        //**********************************************

        double ff = calcFeedForward();
        power = pid + ff;

        if (power > 1) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }
        return power;
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
        return Math.sin(Math.toRadians(temp)) * RC_Telescope.kF;
    }

    public void getCurrent(){
        TelemetryData.telescope_current = this.motor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public boolean isPressed(){
        return this.touch.isPressed();
    }
}
