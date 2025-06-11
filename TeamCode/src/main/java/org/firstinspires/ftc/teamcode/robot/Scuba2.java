package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2 extends Robot {
    MeccanumWheeDrivetrain meccanumWheeDrivetrain;
    Scuba2Arm scuba2Arm;

    public Scuba2(DcMotor rearLeftMotor,
                  DcMotor rearRightMotor,
                  DcMotor frontLeftMotor,
                  DcMotor frontRightMotor,
                  Servo grabberServo,
                  Servo grabberRotateServo,
                  Servo grabberTiltServo,
                  DcMotor leftVerticalSlide,
                  DcMotor rightVerticalSlide,
                  DcMotor horizontalSlide) {
        this.meccanumWheeDrivetrain = new MeccanumWheeDrivetrain(rearRightMotor,
                rearLeftMotor,
                frontRightMotor,
                frontLeftMotor);
        this.scuba2Arm = new Scuba2Arm(grabberTiltServo,
                grabberRotateServo,
                grabberServo,
                leftVerticalSlide,
                rightVerticalSlide,
                horizontalSlide);
    }
    public void move(double... args){
        meccanumWheeDrivetrain.driveTrainControl(args[0], args[1], args[2]);
    }
}
