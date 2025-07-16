package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2Claw {
    private Servo grabberServo;
    private Servo grabberRotateServo;
    private Servo grabberTiltServo;
    public enum Rotation{
        MIDDLE(0.5),
        LEFT(0.75),
        RIGHT(0.25);

        final double rotationPosition;

        Rotation(double rotationPosition) {
            this.rotationPosition = rotationPosition;
        }

        public double getPosition(){
            return rotationPosition;
        }

    }
    public enum Tilt{
        UP(0.6),
        DOWN(0.25);

        final double tiltPosition;

        Tilt(double tiltPosition) {
            this.tiltPosition = tiltPosition;
        }

        public double getPosition(){
            return tiltPosition;
        }

    }

    public Scuba2Claw(Servo grabberTiltServo, Servo grabberRotateServo, Servo grabberServo) {
        this.grabberServo = grabberServo;
        this.grabberRotateServo = grabberRotateServo;
        this.grabberTiltServo = grabberTiltServo;
    }

    public void open(){
        grabberServo.setPosition(0.2);
    }

    public void close(){
        grabberServo.setPosition(0.92);
    }

    public void rotate(Rotation rotation){
        grabberRotateServo.setPosition(rotation.getPosition());
    }

    public void tilt(Tilt tilt){
        grabberTiltServo.setPosition(tilt.getPosition());

    }

}
