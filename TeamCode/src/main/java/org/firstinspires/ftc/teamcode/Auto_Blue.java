package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_AutoBlue;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_AutoRed;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.List;

//@Autonomous(name = "AUTO_BLUE")
public class Auto_Blue extends LinearOpMode{


    List<Recognition> currentRecognitions;

    // @Override
    public void runOpMode() throws InterruptedException {
        boolean fromAuto = true;
        boolean isRed = false;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        GamepadEx myGamePad = new GamepadEx(gamepad1);
        AHRS navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        Launcher launcher = new Launcher(
                hardwareMap.get(Servo.class, "airplane"));
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
                hardwareMap.get(Servo.class, "leftClaw"),
                hardwareMap.get(Servo.class, "rightClaw"),
                hardwareMap.get(TouchSensor.class, "clawSensor"),
                fromAuto);
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        Drive driveTrain = new Drive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx,
                fromAuto,
                false);

        Arm arm = new Arm(shoulder, telescope, claw, lift, wrist, navx, blinkin, fromAuto, isRed);

        Camera visionCam = new Camera(
                hardwareMap.get(WebcamName.class, "Webcam 1")
        );
        visionCam.initTfod();
        DcMotorEx ring = hardwareMap.get(DcMotorEx.class, "ring");


        ElapsedTime timer = new ElapsedTime();
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navx.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        ring.setPower(1);

        telemetry.addData("yaw", TelemetryData.yaw);
        telemetry.addData("x distance", driveTrain.getXDistance());
        telemetry.addData("y distance", driveTrain.getYDistance());
        telemetry.addData("x power", TelemetryData.xPower);
        telemetry.addData("y power", TelemetryData.yPower);
        telemetry.update();
        int castle = 0;
        while (!opModeIsActive()) {
            if(visionCam.returnRecogs().size() != 0) {
                currentRecognitions = visionCam.returnRecogs();
                Recognition recognition = currentRecognitions.get(0);
                if (recognition.getBottom() > RC_AutoBlue.cameraLine) {
                    castle = 0;
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                } else if (recognition.getBottom() < RC_AutoBlue.cameraLine) {
                    castle = 1;
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                telemetry.addData("bottom", recognition.getBottom());
                telemetry.addData("castle", castle);
                telemetry.update();
            }
            else  {
                castle = -1;
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
        waitForStart();

        boolean shoulderWasMoving = false;
        boolean lifting = false;
        boolean autoRun = false;

        int cameraDetect = 0;
        int step = 0;
        int half = 0;
        float objSize = 0;
        boolean autoTargetSet = false;
        boolean stop = false;
        boolean armUpdateOverride = false;
        int lastY = 0;


        ElapsedTime resetTimer = new ElapsedTime();
        boolean startTimer = false;
        arm.initialMove();

        int autoRunStage = 0;
        boolean armCommand = false;
        while (opModeIsActive()) {
            double curX = driveTrain.getXDistance();
            double curY = driveTrain.getYDistance();
            if (autoRunStage == 0) {
                //get away from the wall and the truss
                driveTrain.drive(-.4,-.2,0, true);
                //when far enough away, spin
                if (curX < -50 && curY < -50) {
                    driveTrain.setHeadingToMaintain(-1.57);
                }
                //move on if we have turned more than 80 degrees
                if (TelemetryData.yaw < -80) {
                    autoRunStage = 1;
                }
            } else if (autoRunStage == 1) {
                //move to backboard and prep to drop off
                if (!armCommand) {
                    arm.moveToDropOff();
                    armCommand = true;
                    if (castle == 0) {
                        driveTrain.setTarget(RC_AutoRed.x_center_board, RC_AutoRed.y_center_board);
                    }
                }
                if (driveTrain.update(10,10, .3, .3, -1)) {
                    driveTrain.drive(0,0,0,true);
                    autoRunStage = 2;
                    armCommand = false;
                    resetTimer.reset();
                }
            } else if (autoRunStage == 2 && resetTimer.seconds() > 2) {
                //move arm down before we move
                if (!armCommand) {
                    arm.moveToStow(1);
                    armCommand = true;
                }
                if (TelemetryData.shoulder_position < 500){
                    autoRunStage = 3;
                    armCommand = false;
                }
            } else if (autoRunStage == 3) {
                //move to the spike
                if (!armCommand) {
                    if (castle == 0) {
                        driveTrain.setTarget(RC_AutoRed.x_center_spike, RC_AutoRed.y_center_spike);
                    }
                    arm.moveToPickup(true);
                    armCommand = true;
                }
                if (driveTrain.update(10,10, .3, .3, -1)) {
                    autoRunStage = 4;
                    armCommand = false;
                    resetTimer.reset();
                }
            } else if (autoRunStage == 4) {
                //move to the center
                if (!armCommand) {
                    arm.moveToStow(0);
                    armCommand = true;
                }
                if (resetTimer.seconds() > 2) {
                    driveTrain.drive(0, -.4, 0, true);
                }
                if (driveTrain.getXDistance() < -1300) {
                    //driveTrain.drive(0,0,0, true);
                    autoRunStage = 5;
                    armCommand = false;
                }
            } else if (autoRunStage == 5) {
                //move to the stack
                if (!armCommand) {
                    driveTrain.setTarget(RC_AutoRed.x_stack, RC_AutoRed.y_stack);
                    armCommand = true;
                }
                if (driveTrain.getYDistance() < -200){
                    arm.prepStackAttack();
                }
                if (driveTrain.update(10, 10, .3, .3, -1)) {
                    autoRunStage = 6;
                    armCommand = false;
                    driveTrain.drive(0,0,0, true);
                    arm.stackAttack();
                    resetTimer.reset();
                }
            } else if (autoRunStage == 6 && resetTimer.seconds() > 2) {
                //driveTrain.setToCoast();
                if (!armCommand) {
                    arm.moveToStow(2);
                    driveTrain.setTarget(RC_AutoRed.x_safe, RC_AutoRed.y_safe);
                    armCommand = true;
                }
                if (driveTrain.update(10, 10, .3, .3, -1)) {
                    autoRunStage = 7;
                    armCommand = false;
                }
            } else if (autoRunStage == 7) {
                if (!armCommand) {
                    arm.moveToDropOff();
                    driveTrain.setTarget(RC_AutoRed.x_center_board, RC_AutoRed.y_center_board-10);
                    armCommand = true;
                }
                if (driveTrain.update(10, 10, .3, .3, -1)) {
                    driveTrain.drive(0,0,0, true);
                    autoRunStage = 8;
                    armCommand = false;
                    resetTimer.reset();
                }
            } else if (autoRunStage == 8) {
                if (!armCommand && resetTimer.seconds()>2) {
                    arm.moveToStow(2);
                    armCommand = true;
                    autoRunStage = 9;
                }
            }

            arm.updateEverything();
            telemetry.addData("wrist pos", TelemetryData.wrist_position);
            telemetry.addData("collision detected", TelemetryData.collisionDetected);
            telemetry.addData("yaw", TelemetryData.yaw);
            telemetry.addData("x distance", driveTrain.getXDistance());
            telemetry.addData("y distance", driveTrain.getYDistance());
            telemetry.addData("x power", TelemetryData.xPower);
            telemetry.addData("y power", TelemetryData.yPower);
            telemetry.addData("autoRunStage", autoRunStage);
            telemetry.addData("telescope target", TelemetryData.telescope_target);
            telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("telescope power", TelemetryData.telescope_power);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.update();

        }
        RC_Drive.yaw_from_auto = TelemetryData.yaw;
    }
}
