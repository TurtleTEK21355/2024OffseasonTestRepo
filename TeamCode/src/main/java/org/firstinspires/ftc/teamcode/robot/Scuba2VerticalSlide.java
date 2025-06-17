package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Scuba2VerticalSlide{
    private DcMotor leftVerticalSlide;
    private DcMotor rightVerticalSlide;
    private final double TICKS_PER_REVOLUTION_223RPM = 751.8;
    private final double BOTTOM_LIMIT = 0;
    private final double TOP_LIMIT = 8.1;
    private final double VIPER_SLIDE_LIMIT_BOTTOM = TICKS_PER_REVOLUTION_223RPM * BOTTOM_LIMIT;
    private final double VIPERSLIDE_LIMIT_TOP = TICKS_PER_REVOLUTION_223RPM * TOP_LIMIT;
    private final double IDLE_POWER = 0.1;

    public Scuba2VerticalSlide(DcMotor leftVerticalSlide, DcMotor rightVerticalSlide) {
        this.leftVerticalSlide = leftVerticalSlide;
        this.rightVerticalSlide = rightVerticalSlide;
    }

    public void move(double power){
        if (VIPER_SLIDE_LIMIT_BOTTOM < getPosition() && getPosition() < VIPERSLIDE_LIMIT_TOP){
            leftVerticalSlide.setPower((power)+ IDLE_POWER);
            rightVerticalSlide.setPower((power)+ IDLE_POWER);
        }
        else if (getPosition() > VIPERSLIDE_LIMIT_TOP && power > -0.1){
            leftVerticalSlide.setPower(IDLE_POWER);
            rightVerticalSlide.setPower(IDLE_POWER);
        }
        else if (getPosition() < VIPER_SLIDE_LIMIT_BOTTOM && 0.1 > power){
            leftVerticalSlide.setPower(IDLE_POWER);
            rightVerticalSlide.setPower(IDLE_POWER);
        }
        else{
            leftVerticalSlide.setPower((power)+ IDLE_POWER);
            rightVerticalSlide.setPower((power)+ IDLE_POWER);
        }
    }
    
    public double getPosition(){
        return ((leftVerticalSlide.getCurrentPosition()+rightVerticalSlide.getCurrentPosition())/2.0);
    }
}
