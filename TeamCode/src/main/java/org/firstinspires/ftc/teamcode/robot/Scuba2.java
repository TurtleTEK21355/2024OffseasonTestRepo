package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Scuba2 {
    public MeccanumWheeDrivetrain drivetrain;
    public Scuba2Arm arm;

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
        this.drivetrain = new MeccanumWheeDrivetrain(rearRightMotor,
                rearLeftMotor,
                frontRightMotor,
                frontLeftMotor);
        this.arm = new Scuba2Arm(grabberTiltServo,
                grabberRotateServo,
                grabberServo,
                leftVerticalSlide,
                rightVerticalSlide,
                horizontalSlide);
    }
}
