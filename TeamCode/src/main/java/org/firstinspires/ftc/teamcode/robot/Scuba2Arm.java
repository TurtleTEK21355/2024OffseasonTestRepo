package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2Arm{
    public Scuba2HorizontalSlide linearActuator;
    public Scuba2VerticalSlide viperSlides;
    public Scuba2Claw claw;

    public Scuba2Arm(Servo grabberTiltServo,
                     Servo grabberRotateServo,
                     Servo grabberServo,
                     DcMotor leftVerticalSlide,
                     DcMotor rightVerticalSlide,
                     DcMotor horizontalSlide){
        this.linearActuator = new Scuba2HorizontalSlide(horizontalSlide);
        this.viperSlides = new Scuba2VerticalSlide(leftVerticalSlide, rightVerticalSlide);
        this.claw = new Scuba2Claw(grabberTiltServo, grabberRotateServo, grabberServo);
    }
}
