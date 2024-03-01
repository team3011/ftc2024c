package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Launcher;

public class Launcher {
    public Servo servo;

    /**
     * Class constructor
     *
     * @param s servo obj
     */
    public Launcher(Servo s) {
        this.servo = s;
        this.servo.setPosition(RC_Launcher.hold);
    }

    public void launchIt() {
        this.servo.setPosition(RC_Launcher.go);
    }
}
