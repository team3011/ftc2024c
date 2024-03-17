package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.RobotConstants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.Telescope;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

import java.text.DecimalFormat;


@Autonomous(name = "SubSystem_Test")
public class SubSystem_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean fromAuto = true;
        boolean isRed = true;

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        AHRS navx = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        Launcher launcher = new Launcher(
                hardwareMap.get(Servo.class,"airplane"));
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"),
                true);
        Telescope telescope = new Telescope(
                hardwareMap.get(DcMotorEx.class, "telescope"),
                hardwareMap.get(TouchSensor.class, "telescopeSensor"),
                true);
        Wrist wrist = new Wrist(
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"),
                true);
        Lift lift = new Lift(
                hardwareMap.get(DcMotorEx.class, "lift"));
        Claw claw = new Claw(
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"),
                hardwareMap.get(TouchSensor.class,"clawSensor"),
                false);
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        Drive driveTrain = new Drive(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx,
                true,
                true);

        Arm arm = new Arm(shoulder, telescope, claw, lift, wrist, navx, blinkin, fromAuto, isRed);

        boolean connected = navx.isConnected();
        telemetry.addData("1 navX-Device", connected ?
                "Connected" : "Disconnected" );
        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");



        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        waitForStart();
        while(opModeIsActive()) {
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);


            if ( connected ) {
                gyrocal = (navx.isCalibrating() ?
                        "CALIBRATING" : "Calibration Complete");
                magcal = (navx.isMagnetometerCalibrated() ?
                        "Calibrated" : "UNCALIBRATED");
                yaw = df.format(navx.getYaw());
                pitch = df.format(navx.getPitch());
                roll = df.format(navx.getRoll());
                ypr = yaw + ", " + pitch + ", " + roll;
                compass_heading = df.format(navx.getCompassHeading());
                fused_heading = df.format(navx.getFusedHeading());
                if (!navx.isMagnetometerCalibrated()) {
                    compass_heading = "-------";
                }
                cf = compass_heading + ", " + fused_heading;
                if ( navx.isMagneticDisturbance()) {
                    cf += " (Mag. Disturbance)";
                }
                motion = (navx.isMoving() ? "Moving" : "Not Moving");
                if ( navx.isRotating() ) {
                    motion += ", Rotating";
                }
            } else {
                gyrocal =
                        magcal =
                                ypr =
                                        cf =
                                                motion = "-------";
            }
            telemetry.addData("2 GyroAccel", gyrocal );
            telemetry.addData("3 Y,P,R", ypr);
            telemetry.addData("4 Magnetometer", magcal );
            telemetry.addData("5 Compass,9Axis", cf );
            telemetry.addData("6 Motion", motion);
            //launcher.launchIt();
            //shoulder.update();
            //telescope.manualMove(left_y);
            //telescope.update();
            lift.moveManual(right_y);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("telescope power", TelemetryData.telescope_power);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.update();

        }

    }

    /**
     * removes the analog drift
     * @param input
     * @return
     */
    private double zeroAnalogInput(double input){
        if (Math.abs(input) < RobotConstants.analogTol){
            input = 0;
        } else if (input > 0) {
            input -= RobotConstants.analogTol;
        } else {
            input += RobotConstants.analogTol;
        }
        return input;
    }
}
