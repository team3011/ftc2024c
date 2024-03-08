package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
@Autonomous(name = "AUTO_RED")
public class Auto_Red extends LinearOpMode{


    List<Recognition> currentRecognitions;

    // @Override
    public void runOpMode() throws InterruptedException {
        boolean fromAuto = true;
        boolean isRed = true;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        GamepadEx myGamePad = new GamepadEx(gamepad1);
        NavxMicroNavigationSensor navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
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
                fromAuto);

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

        ring.setPower(0);

        waitForStart();

        boolean shoulderWasMoving = false;
        boolean lifting = false;
        boolean autoRun = false;
        int autoRunStage = 0;
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
        while (opModeIsActive()) {
            arm.updateEverything();
            telemetry.addData("telescope target", TelemetryData.telescope_target);
            telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("telescope power", TelemetryData.telescope_power);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.update();
        }
    }
}
