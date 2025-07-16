package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Scuba2HorizontalSlide{
    private DcMotor horizontalSlide;

    public Scuba2HorizontalSlide(DcMotor horizontalSlide) {
        this.horizontalSlide = horizontalSlide;
        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void move(double power){
        horizontalSlide.setPower(power);
    }
}
