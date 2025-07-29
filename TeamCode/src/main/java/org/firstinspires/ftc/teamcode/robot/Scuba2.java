package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2 {
    public MeccanumWheeDrivetrain drivetrain;
    public Scuba2Arm arm;

    public Scuba2(
                  Servo grabberServo,
                  Servo grabberRotateServo,
                  Servo grabberTiltServo,
                  DcMotor leftVerticalSlide,
                  DcMotor rightVerticalSlide,
                  DcMotor horizontalSlide
    ) {
        this.drivetrain = new MeccanumWheeDrivetrain();
        this.arm = new Scuba2Arm(
                grabberTiltServo,
                grabberRotateServo,
                grabberServo,
                leftVerticalSlide,
                rightVerticalSlide,
                horizontalSlide
        );
    }
    public void addDrivetrainMotor(DcMotor motor, MeccanumWheeDrivetrain.WheelPosition wheelPosition){
        drivetrain.addMotor(motor, wheelPosition);

    }
    public void addViperslideMotor(DcMotor motor){

    }
    public void addOtos(SparkFunOTOS otosSensor){
        drivetrain.addOtos(otosSensor);
    }
    public void addClawServo(){

    }
}
