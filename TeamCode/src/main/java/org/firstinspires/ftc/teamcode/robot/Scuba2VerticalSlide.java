package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Scuba2VerticalSlide{
    private final DcMotor leftVerticalSlide;
    private final DcMotor rightVerticalSlide;
    private final TouchSensor viperSlideSensor;
    private final double TICKS_PER_REVOLUTION_223RPM = 751.8;
    private final double BOTTOM_LIMIT = 0;
    private final double TOP_LIMIT = 2.3;
    private final double VIPER_SLIDE_LIMIT_BOTTOM = TICKS_PER_REVOLUTION_223RPM * BOTTOM_LIMIT;
    private final double VIPERSLIDE_LIMIT_TOP = 1740;
    private final double IDLE_POWER = 0.1;
    private final double Kp = 0.09;
    private final double Ki = 0;
    private final double Kd = 0;
    private double viperSlidePosition = 0;
    private double lastRawSlidePosition = 0;
    private boolean viperSlideFullyInitialized = false;

    public Scuba2VerticalSlide(DcMotor leftVerticalSlide, DcMotor rightVerticalSlide, TouchSensor viperSlideSensor) {
        this.leftVerticalSlide = leftVerticalSlide;
        this.rightVerticalSlide = rightVerticalSlide;
        this.viperSlideSensor = viperSlideSensor;
        leftVerticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightVerticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlidePosition = getPosition();
    }

    public void movePower(double power){
        viperSlidePosition += getPosition() - lastRawSlidePosition;
        if (viperSlideSensor.getValue() > 0.9 && 0.1 > power){
            viperSlidePosition = 0.0;
            viperSlideFullyInitialized = true;
            leftVerticalSlide.setPower(IDLE_POWER);
            rightVerticalSlide.setPower(IDLE_POWER);
        }
        else if (viperSlidePosition > VIPERSLIDE_LIMIT_TOP && power > -0.1){
            if (viperSlideFullyInitialized){
                leftVerticalSlide.setPower(IDLE_POWER);
                rightVerticalSlide.setPower(IDLE_POWER);
            } else {
                leftVerticalSlide.setPower(((power)+ IDLE_POWER));
                rightVerticalSlide.setPower(((power)+ IDLE_POWER));
            }
        }
        else {
            leftVerticalSlide.setPower(((power)+ IDLE_POWER));
            rightVerticalSlide.setPower(((power)+ IDLE_POWER));
        }
        lastRawSlidePosition = getPosition();
    }

    private void movePosition(double targetPos, double MaxSpeed) {
        double previousError = 0;
        double integral = 0;
        double current = getPosition();
        double error = targetPos - current;

        while (!(Math.abs(error) <= 0.1)) {
            current = getPosition();

            error = targetPos - current;

            integral = integral + error;

            double derivative = error - previousError;

            double Power = Math.min((Kp * error) + (Ki * integral) + (Kd * derivative), Math.abs(MaxSpeed));

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
    public String getStats(){
        return "viperSlidePosition: " + viperSlidePosition + "\nIsFullyInitialized: " + viperSlideFullyInitialized + "\ntouchSensor: " + viperSlideSensor.getValue();
    }
}
