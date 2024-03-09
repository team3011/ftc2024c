package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;


// excellent resource on programming mecanum wheels
// https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric-final-sample-code
@TeleOp(name = "TELE_RED")
public class Tele_Red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean fromAuto = false;
        boolean isRed = true;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        GamepadEx myGamePad = new GamepadEx(gamepad1);
        AHRS navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        Launcher launcher = new Launcher(
                hardwareMap.get(Servo.class,"airplane"));
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"),
                fromAuto);
        Telescope telescope = new Telescope(
                hardwareMap.get(DcMotorEx.class, "telescope"),
                hardwareMap.get(TouchSensor.class, "telescopeSensor"),
                fromAuto);
        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"),
                fromAuto);
        Lift lift = new Lift(
                hardwareMap.get(DcMotorEx.class, "lift"));
        Claw claw = new Claw(
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"),
                hardwareMap.get(TouchSensor.class,"clawSensor"),
                fromAuto);
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        Drive driveTrain = new Drive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx,
                fromAuto);

        Arm arm = new Arm(shoulder, telescope, claw, lift, wrist, navx, blinkin, fromAuto, isRed);

        ElapsedTime timer = new ElapsedTime();
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navx.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        waitForStart();

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        double left_t = gamepad1.left_trigger;
        double right_t = gamepad1.right_trigger;
        boolean a_state = false;
        boolean b_state = false;
        boolean x_state = false;
        boolean y_state = false;
        boolean shoulderWasMoving = false;
        boolean lifting = false;
        boolean endGameStarted = false;


        while(opModeIsActive()){
            myGamePad.readButtons();
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);
            left_t = -zeroAnalogInput(gamepad1.left_trigger);
            right_t = zeroAnalogInput(gamepad1.right_trigger);

            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.A)) {
                arm.moveToPickup(false);
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.X)) {
                arm.moveToStow(2);
                TelemetryData.liftStage = -3;
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.Y)) {
                arm.moveToDropOff();
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.Y)) {
                driveTrain.setHeadingToMaintain(0);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.X)) {
                driveTrain.setHeadingToMaintain(-1.57);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.A)) {
                driveTrain.setHeadingToMaintain(3.14);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.B)) {
                driveTrain.setHeadingToMaintain(1.57);
            }

            if (left_t != 0) {
                endGameStarted = true;
                shoulderWasMoving = true;
                arm.prepForLift(left_t);
            } else if (shoulderWasMoving) {
                shoulderWasMoving = false;
                arm.prepForLift(0);
            }
            if (right_t != 0) {
                lifting = true;
                arm.lifting(right_t);
            } else if (lifting) {
                lifting = false;
                arm.lifting(0);
            }

            driveTrain.drive(RC_Drive.red_leftXInverse * left_x, RC_Drive.red_leftYInverse * left_y, RC_Drive.red_rightXInverse * right_x, false);

            if (!endGameStarted) {
                if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    arm.manualMoveB(-right_y);
                } else {
                    arm.manualMoveA(-right_y);
                }
                arm.updateEverything();
            }

            telemetry.addData("x distance", driveTrain.getXDistance());
            telemetry.addData("y distance", driveTrain.getYDistance());
            telemetry.addData("yaw", TelemetryData.yaw);
            telemetry.addData("yaw offset from auto", RC_Drive.yaw_from_auto);
            telemetry.addData("wrist position", TelemetryData.wrist_position);
            telemetry.addData("maintain heading of", TelemetryData.whatHeadingDo);
            telemetry.addData("telescope target", TelemetryData.telescope_target);
            telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            //telemetry.addData("lift stage", TelemetryData.liftStage);
            //telemetry.addData("shoulder current",TelemetryData.shoulder_current);
            //telemetry.addData("telescope current",TelemetryData.telescope_current);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            //telemetry.addData("telescope power", TelemetryData.telescope_power);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            //telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.update();

        }

        driveTrain.drive(0,0,0, false);
    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input)<0.1){
            input = 0;
        } else if (input < 0) {
            input += .1;
        } else if (input > 0) {
            input -= .1;
        }
        return input;
    }
}