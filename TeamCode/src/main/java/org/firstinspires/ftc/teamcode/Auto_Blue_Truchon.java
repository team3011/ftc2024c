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
import org.firstinspires.ftc.teamcode.RobotConstants.RC_AutoBlue;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Point;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.util.List;

@Autonomous(name = "AUTO_BLUE_TRUCHON")
public class Auto_Blue_Truchon extends LinearOpMode{


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
                false
        );


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

        int castle = 0;
        Point board = new Point(0, 0);
        Point spike = new Point(0, 0);
        while (!opModeIsActive()) {
            telemetry.addData("bottom", 999);
            if(!visionCam.returnRecogs().isEmpty()) {
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
            }
            else  {
                castle = -1;
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            telemetry.addData("castle", castle);
            telemetry.update();
            arm.initialMove();
            arm.updateEverything();
        }

        if (castle == 0) {
            board.setXY(RC_AutoBlue.x_center_board, RC_AutoBlue.y_center_board);
            spike.setXY(RC_AutoBlue.x_center_spike, RC_AutoBlue.y_center_spike);
        }
        if (castle == 1) {
            board.setXY(RC_AutoBlue.x_right_board, RC_AutoBlue.y_right_board);
            spike.setXY(RC_AutoBlue.x_right_spike, RC_AutoBlue.y_right_spike);
        }
        if (castle == -1) {
            board.setXY(RC_AutoBlue.x_left_board, RC_AutoBlue.y_left_board);
            spike.setXY(RC_AutoBlue.x_left_spike, RC_AutoBlue.y_left_spike);
        }


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

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        int autoRunStage = 0;
        boolean armCommand = false;
        while (opModeIsActive()) {
            double curX = driveTrain.getXDistance();
            double curY = driveTrain.getYDistance();
            if (autoRunStage == 0) {
                //get away from the wall and the truss
                driveTrain.drive(.4,-.2,0, true);
                //when far enough away, spin
                if (curX > 50 && curY < -50) {
                    driveTrain.setHeadingToMaintain(1.57);
                }
                //move on if we have turned more than 80 degrees
                if (TelemetryData.yaw > 80) {
                    autoRunStage = 1;
                    //driveTrain.drive(0,0,0, true);
                    //driveTrain.setToCoast();
                }
            } else if (autoRunStage == 1) {
                //move to backboard and prep to drop off
                if (!armCommand) {
                    arm.autoMoveToDropOff();
                    armCommand = true;
                    driveTrain.setTarget(board.returnX(), board.returnY());
                    //driveTrain.setTarget(844,966);
                }
                if (driveTrain.updateV2(15,15, .2, .2, -1)) {
                    driveTrain.drive(0,0,0,true);
                    autoRunStage = 2;
                    armCommand = false;
                    resetTimer.reset();
                    //driveTrain.setToCoast();
                }
            } else if (autoRunStage == 2 && resetTimer.seconds() > 2) {
                //move arm down before we move
                if (!armCommand) {
                    //arm.moveToStow(1);
                    arm.autoStow();
                    armCommand = true;

                }

                if (TelemetryData.shoulder_position < 10) {
                    //driveTrain.drive(0,0,0,true);
                    //driveTrain.setToCoast();
                    arm.autoMoveToPickup();
                    autoRunStage = 3;
                    armCommand = false;
                    resetTimer.reset();
                }
            } else if (autoRunStage == 3 && resetTimer.seconds()>2) {
                //move to the spike
                if (!armCommand) {
                    driveTrain.setTarget(spike.returnX(), spike.returnY());
                    /*
                    if (castle == 1) {

                        arm.moveToPickupTruchon(true);
                    }
                    else {
                        arm.moveToPickup(true);
                    }

                     */
                    armCommand = true;
                }
                if (driveTrain.updateV2(10,10, .2, .2, -1) && resetTimer.seconds() > 0.5) {
                    driveTrain.drive(0,0,0,true);
                    autoRunStage = 4;
                    armCommand = false;
                    resetTimer.reset();
                }
            } else if (autoRunStage == 4) {
                //move to the center
                if (!armCommand) {
                    arm.moveToStow(0);
                    armCommand = true;
                    driveTrain.setTarget(board.returnX()+500, board.returnY()-100);
                }
                if (driveTrain.updateV2(10,10, .2, .2, -1)) {
                    driveTrain.drive(0,0,0,true);
                    autoRunStage = 5;
                    armCommand = false;
                    resetTimer.reset();
                }
            }

                /*
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
                    driveTrain.setTarget(RC_AutoBlue.x_stack, RC_AutoBlue.y_stack);
                    armCommand = true;
                }
                if (driveTrain.getYDistance() < -200){
                    arm.prepStackAttack();
                }
                if (driveTrain.updateV2(10, 10, .3, .3, -1)) {
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
                    driveTrain.setTarget(RC_AutoBlue.x_safe, RC_AutoBlue.y_safe);
                    armCommand = true;
                }
                if (driveTrain.updateV2(10, 10, .3, .3, -1)) {
                    autoRunStage = 7;
                    armCommand = false;
                }
            } else if (autoRunStage == 7) {
                if (!armCommand) {
                    arm.moveToDropOff();
                    driveTrain.setTarget(RC_AutoBlue.x_center_board, RC_AutoBlue.y_center_board);
                    armCommand = true;
                }
                if (driveTrain.updateV2(10, 10, .3, .3, -1)) {
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

                 */

            arm.updateEverything();
           // telemetry.addData("wrist pos", TelemetryData.wrist_position);
           // telemetry.addData("collision detected", TelemetryData.collisionDetected);
           telemetry.addData("yaw", TelemetryData.yaw);
            telemetry.addData("x distance", curX);
            telemetry.addData("y distance", curY);
            telemetry.addData("x_power", TelemetryData.xPower);
            telemetry.addData("y_power", TelemetryData.yPower);
           // telemetry.addData("x power", TelemetryData.xPower);
            //telemetry.addData("y power", TelemetryData.yPower);
            telemetry.addData("autoRunStage", autoRunStage);
           telemetry.addData("telescope target", TelemetryData.telescope_target);
           telemetry.addData("shoulder target", TelemetryData.shoulder_target);
           telemetry.addData("telescope position", TelemetryData.telescope_position);
           // telemetry.addData("telescope power", TelemetryData.telescope_power);
           telemetry.addData("shoulder position", TelemetryData.shoulder_position);
           // telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            //telemetry.addData("x diff", driveTrain.getDiffX());
            //telemetry.addData("y diff", driveTrain.getDiffY());
            telemetry.update();

        }
        RC_Drive.yaw_from_auto = TelemetryData.yaw;
    }
}
