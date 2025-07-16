package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.text.Format;

public class MeccanumWheeDrivetrain extends Drivetrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private SparkFunOTOS otosSensor;

    public enum WheelPosition{
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT;
    }

    public void addMotor(DcMotor motor, WheelPosition wheelPosition){
        switch(wheelPosition){
            case FRONT_LEFT:
                this.frontLeftMotor = motor;
                this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            case FRONT_RIGHT:
                this.frontRightMotor = motor;
                this.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            case REAR_LEFT:
                this.rearLeftMotor = motor;
                this.rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            case REAR_RIGHT:
                this.rearRightMotor = motor;
                this.rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        }

    }

    public void addOtos(SparkFunOTOS otosSensor){
        this.otosSensor = otosSensor;
    }

    public void move(double drive, double strafe, double turn) {
        frontLeftMotor.setPower(Range.clip(drive - strafe + turn, -1, 1));
        frontRightMotor.setPower(Range.clip(drive - strafe - turn, -1, 1));
        rearLeftMotor.setPower(Range.clip(drive + strafe + turn, -1, 1));
        rearRightMotor.setPower(Range.clip(drive + strafe - turn, -1, 1));

    }

}
