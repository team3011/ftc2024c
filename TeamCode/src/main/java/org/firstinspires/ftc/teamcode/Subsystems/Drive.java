package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Drive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    NavxMicroNavigationSensor navX;
    IntegratingGyroscope gyro;
    private double headingToMaintain = 0;
    private String whatHeadingDo;

    public Drive(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, NavxMicroNavigationSensor n, boolean fromAuto) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.navX = n;
        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.gyro = (IntegratingGyroscope) this.navX;

        if (fromAuto) {
            this.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            this.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        this.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * RUNS A TEST OF THE MOTORS
     *
     * @param delay how long do you want to run each motor for in ms
     * @param power at what speed
     * @throws InterruptedException
     */
    public void testMotors(int delay, double power) throws InterruptedException {
        this.frontLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(power);
        Thread.sleep(delay);
        this.frontRight.setPower(0);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.backLeft.setPower(0);
        this.backRight.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);

    }

    private int convertToMM(int input){
        return (int) (input*0.15073/2);
    }
}