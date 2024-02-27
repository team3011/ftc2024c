package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;

@TeleOp(name = "SubSystem_Test")
public class SubSystem_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"),
                true);
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

            shoulder.update();
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            telemetry.addData("shoulder velocity", TelemetryData.shoulder_velocity);
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
