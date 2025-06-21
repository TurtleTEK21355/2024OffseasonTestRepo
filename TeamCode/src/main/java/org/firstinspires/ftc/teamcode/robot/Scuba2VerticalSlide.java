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
    private double Kp = 0.09;
    private double Ki = 0;
    private double Kd = 0;

    public Scuba2VerticalSlide(DcMotor leftVerticalSlide, DcMotor rightVerticalSlide) {
        this.leftVerticalSlide = leftVerticalSlide;
        this.rightVerticalSlide = rightVerticalSlide;
    }
    public void movePosition(double position){

    }
    public void movePower(double power){
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
    private void positionControl(double targetPos, double MaxSpeed) {
        double previousError = 0;
        double integral = 0;
        double speed = MaxSpeed;

        double current = getPosition();
        double error = targetPos - current;

        while (!(Math.abs(error) <= 0.1)) {
            current = getPosition();

            error = targetPos - current;

            integral = integral + error;

            double derivative = error - previousError;

            double Power = Math.min((Kp * error) + (Ki * integral) + (Kd * derivative), Math.abs(speed));

            previousError = error;

            leftVerticalSlide.setPower(Power);
            rightVerticalSlide.setPower(Power);
        }

        leftVerticalSlide.setPower(0);
        rightVerticalSlide.setPower(0);
    }

    public double getPosition(){
        return ((leftVerticalSlide.getCurrentPosition()+rightVerticalSlide.getCurrentPosition())/2.0);
    }
}
