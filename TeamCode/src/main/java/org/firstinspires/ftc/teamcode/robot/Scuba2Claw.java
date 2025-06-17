package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2Claw {
    private Servo grabberServo;
    private Servo grabberRotateServo;
    private Servo grabberTiltServo;
    public enum Rotation{
        MIDDLE,
        LEFT,
        RIGHT
    }
    public enum Tilt{
        UP,
        DOWN
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
    public void rotate(Rotation position){

        switch (position) {
            case MIDDLE: grabberRotateServo.setPosition(0.5);
            case LEFT: grabberRotateServo.setPosition(0.75);
            case RIGHT: grabberRotateServo.setPosition(0.25);
        }

    }
    public void tilt(Tilt position){
        switch (position) {
            case UP: grabberTiltServo.setPosition(0.6);
            case DOWN: grabberTiltServo.setPosition(0.25);
        }
    }
}
