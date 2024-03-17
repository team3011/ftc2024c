package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive.last_world_linear_accel_x;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

import java.util.Locale;

public class Drive {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    //NavxMicroNavigationSensor navX;
    //IntegratingGyroscope gyro;
    AHRS navX;
    private double headingToMaintain = 0;
    PIDController x_controller;
    PIDController y_controller;
    double x_lastPIDCalc = 0.01;
    double y_lastPIDCalc = 0.01;
    int lastY = 0;

    public Drive(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, AHRS n, boolean fromAuto, boolean isRed) {
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

        TelemetryData.collisionDetected = false;
        if (fromAuto) {
            this.x_controller = new PIDController(RC_Drive.x_kP, RC_Drive.x_kI, RC_Drive.x_kD);
            this.y_controller = new PIDController(RC_Drive.y_kP, RC_Drive.y_kI, RC_Drive.y_kD);
            this.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            this.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            if (isRed) {
                setHeadingToMaintain(Math.toRadians(RC_Drive.yaw_from_auto));
            } else {
                setHeadingToMaintain(Math.toRadians(-RC_Drive.yaw_from_auto));
            }
        }
        this.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setToCoast(){
        this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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
    public void drive(double leftStickX, double leftStickY, double rightStickX, boolean fromAuto) {
        double x = leftStickX;
        double y = leftStickY; // Counteract imperfect strafing
        double rx = rightStickX * RC_Drive.rotation_multi; //what way we want to rotate

        if (fromAuto) {
            TelemetryData.yaw = Math.round(this.navX.getYaw() * 10) / 10.0;
        } else {
            TelemetryData.yaw = Math.round(this.navX.getYaw() * 10) / 10.0 - 90;
        }
        double robotHeading = Math.toRadians(TelemetryData.yaw);

        double curr_world_linear_accel_x = this.navX.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - RC_Drive.last_world_linear_accel_x;
        RC_Drive.last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = this.navX.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - RC_Drive.last_world_linear_accel_y;
        RC_Drive.last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > 0.5 ) ||
                ( Math.abs(currentJerkY) > 0.5) ) {
            TelemetryData.collisionDetected = true;
        }

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
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeading;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double rotY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

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
        return -result * RC_Drive.autoRotation_multi;
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

    public void setTarget(int mmX, int mmY){
        TelemetryData.xTarget = mmX;
        TelemetryData.yTarget = mmY;
    }

    public int getXDistance(){
        return convertToMM(this.backLeft.getCurrentPosition());
    }

    public int getYDistance(){
        return convertToMM(this.backRight.getCurrentPosition());
    }

    public int returnXTarget() {
        return TelemetryData.xTarget;
    }
    public int returnYTarget() {
        return TelemetryData.yTarget;
    }

    public int getDiffX() { return Math.abs(getXDistance() - returnXTarget()); }
    public int getDiffY() { return Math.abs(getYDistance() - returnYTarget()); }

    public boolean update(int xTol, int yTol, double xMax, double yMax, int inverse){
        this.x_controller.setPID(RC_Drive.x_kP, RC_Drive.x_kI, RC_Drive.x_kD);
        this.y_controller.setPID(RC_Drive.y_kP, RC_Drive.y_kI, RC_Drive.y_kD);
        double x_power = limiter(calcXPower(),xMax) * 1.3;
        double y_power = inverse * limiter(calcYPower(),yMax)  * 1.3;
        TelemetryData.xPower = x_power;
        TelemetryData.yPower = y_power;
        drive(-y_power, x_power, 0, true);
        return Math.abs(x_power) < .1 && Math.abs(y_power) < .1;
    }

    public boolean updateV2(int xTol, int yTol, double xMax, double yMax, int inverse) {
        double x_power = 0;
        double y_power = 0;
        int curX = getXDistance();
        int curY = getYDistance();


        this.x_controller.setPID(RC_Drive.x_kP, RC_Drive.x_kI, RC_Drive.x_kD);
        this.y_controller.setPID(RC_Drive.y_kP, RC_Drive.y_kI, RC_Drive.y_kD);
        double x_power2 = Math.abs(limiter(calcXPower(),xMax)) + .1;
        double y_power2 = Math.abs(limiter(calcYPower(),yMax)) + .1;

        //double x_power2 = xMax;
        //double y_power2 = yMax;
        //TelemetryData.xPower = xMax;
        //TelemetryData.yPower = yMax;

        if (inverse == 1) {
            //this is for red
            if ( Math.abs(TelemetryData.xTarget - curX) > xTol) {
                if (TelemetryData.xTarget > getXDistance()) {
                    x_power = x_power2;
                } else {
                    x_power = -x_power2;
                }
            }
            if (Math.abs(TelemetryData.yTarget - curY)  > yTol) {
                if (TelemetryData.yTarget > getYDistance()) {
                    y_power = -y_power2;
                } else {
                    y_power = y_power2;
                }
            }
        } else {
            //this is for blue
            if ( Math.abs(TelemetryData.xTarget - curX) > xTol) {
                if (TelemetryData.xTarget > getXDistance()) {
                    x_power = -x_power2;
                } else {
                    x_power = x_power2;
                }
            }
            if (Math.abs(TelemetryData.yTarget - curY)  > yTol) {
                if (TelemetryData.yTarget > getYDistance()) {
                    y_power = y_power2;
                } else {
                    y_power = -y_power2;
                }
            }
        }
        drive(y_power,x_power,0,true);
        boolean exit = false;
        if (Math.abs(getYDistance() - this.lastY)  < 5) {
            exit = true;
        } else {
            this.lastY = getYDistance();
        }
        return (x_power == 0 && y_power == 0);
    }

    private double limiter(double input, double lim) {
        //this will limit the pid to a range of -1 to 1
        if (input > lim) {
            input = lim;
        } else if (input < -lim) {
            input = -lim;
        }
        return input;
    }
    private double calcXPower(){
        double power = 0;
        double pid = this.x_controller.calculate(getXDistance(), TelemetryData.xTarget);

        pid = limiter(pid / 10,1);
        return pid;
        /*
        //***********************************************
        //this section reduces the acceleration by reducing how fast the pid calculation can change
        if (this.x_lastPIDCalc > 0) {
            if (pid > 0) {
                if (pid - this.x_lastPIDCalc > .2) {
                    pid = this.x_lastPIDCalc * RC_Drive.x_velMultiplier;
                }
            } else {
                pid = -0.01;
            }
        } else {
            if (pid < 0) {
                if (pid - this.x_lastPIDCalc < -0.2) {
                    pid = this.x_lastPIDCalc * RC_Drive.x_velMultiplier;
                }

            } else {
                pid = 0.01;
            }
        }
        this.x_lastPIDCalc = pid;
        //**********************************************
        pid = limiter(pid / 10);
        return power;

         */
    }
    private double calcYPower(){
        double power = 0;
        double pid = this.y_controller.calculate(getYDistance(), TelemetryData.yTarget);
        pid = limiter(pid / 10,1);
        return pid;
        /*

        //***********************************************
        //this section reduces the acceleration by reducing how fast the pid calculation can change
        if (this.y_lastPIDCalc > 0) {
            if (pid > 0) {
                if (pid - this.y_lastPIDCalc > .2) {
                    pid = this.y_lastPIDCalc * RC_Drive.y_velMultiplier;
                }
            } else {
                pid = -0.01;
            }
        } else {
            if (pid < 0) {
                if (pid - this.y_lastPIDCalc < -0.2) {
                    pid = this.y_lastPIDCalc * RC_Drive.x_velMultiplier;
                }

            } else {
                pid = 0.01;
            }
        }
        this.x_lastPIDCalc = pid;
        //**********************************************
        pid = limiter(pid / 10);
        return power;

         */
    }
}