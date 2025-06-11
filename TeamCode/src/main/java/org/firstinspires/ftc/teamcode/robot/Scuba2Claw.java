package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2Claw {
    private Servo grabberServo;
    private Servo grabberRotateServo;
    private Servo grabberTiltServo;

    public Scuba2Claw(Servo grabberTiltServo, Servo grabberRotateServo, Servo grabberServo) {
        this.grabberServo = grabberServo;
        this.grabberRotateServo = grabberRotateServo;
        this.grabberTiltServo = grabberTiltServo;
    }
}
