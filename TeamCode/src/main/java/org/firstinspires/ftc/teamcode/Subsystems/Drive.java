package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

import java.util.Locale;

public class Drive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    NavxMicroNavigationSensor navX;
    IntegratingGyroscope gyro;
    private double headingToMaintain = 0;

    public Drive(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, NavxMicroNavigationSensor n, boolean fromAuto) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.navX = n;
        this.frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.backRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.gyro = (IntegratingGyroscope) this.navX;

        if (fromAuto) {
            this.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            this.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            setHeadingToMaintain(Math.toRadians(RC_Drive.yaw_from_auto));
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

    /** main thread for this class, commands the motors to do required movements
     *
     * @param leftStickX commands robot strafing movements
     * @param leftStickY commands robot forward and backward movements
     * @param rightStickX commands robot's rotation
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX) {
        double x = leftStickX;
        double y = leftStickY; // Counteract imperfect strafing
        double rx = rightStickX * RC_Drive.rotation_multi; //what way we want to rotate

        Orientation angles = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        TelemetryData.yaw = Math.round(angles.firstAngle * 10) / 10.0 + RC_Drive.yaw_from_auto;
        double robotHeading = Math.toRadians(TelemetryData.yaw);

        if(rx == 0){ //this means that we're trying to maintain our current heading
            double shorter = this.figureOutWhatIsShorter(robotHeading);
            //prevents the motors from working when they realistically cant
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) < RC_Drive.ANGULAR_TOLERANCE;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                rx = shorter; //retrieves direction, magnitude overwritten
                rx = this.setToMinimumTurningSpeed(rx); //overwrites magnitude to minimum speed
            }

            TelemetryData.whatHeadingDo = headingToMaintain;
                    //String.format(Locale.getDefault(),"Maintaining Desired Heading of %.2f",
                    //        headingToMaintain);
        }else{
            //TelemetryData.whatHeadingDo = "Turning";
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeading;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
        double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx)  / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx)   / denominator;
        double backRightPower = (rotY + rotX - rx)  / denominator;

        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);
    }

    /** Determines what direction would be shorter to turn in when trying to maintain our current
     *  heading.
     * @return the shorter difference in heading
     */
    public double figureOutWhatIsShorter(double reading) {
        double result;
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 2 * Math.PI;
        } else {
            oppositeButEqualReading = reading + 2 * Math.PI;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);

        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return result;
    }

    /** changes our current turning speed to a turning speed that allows us to rotate
     *
     * @param rx our current turning speed
     * @return modified turning speed
     */
    private double setToMinimumTurningSpeed(double rx){

        if(Math.abs(rx) < RC_Drive.MINIMUM_TURNING_SPEED) {
            if (rx < 0) {
                return -RC_Drive.MINIMUM_TURNING_SPEED;
            } else {
                return RC_Drive.MINIMUM_TURNING_SPEED;
            }
        }else{
            return rx;
        }
    }

    /**
     *
     * @param input in radians
     */
    public void setHeadingToMaintain(double input){ this.headingToMaintain = input; }
}