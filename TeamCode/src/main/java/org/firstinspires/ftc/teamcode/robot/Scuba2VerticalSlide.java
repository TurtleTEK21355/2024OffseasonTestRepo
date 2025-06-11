package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Scuba2VerticalSlide extends VerticalSlide{
    private DcMotor leftVerticalSlide;
    private DcMotor rightVerticalSlide;

    public Scuba2VerticalSlide(DcMotor leftVerticalSlide, DcMotor rightVerticalSlide) {
        this.leftVerticalSlide = leftVerticalSlide;
        this.rightVerticalSlide = rightVerticalSlide;
    }

}
