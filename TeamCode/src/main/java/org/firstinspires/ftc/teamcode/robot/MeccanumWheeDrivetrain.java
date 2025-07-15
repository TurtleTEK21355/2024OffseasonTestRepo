package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MeccanumWheeDrivetrain extends Drivetrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private SparkFunOTOS otosSensor;

    public MeccanumWheeDrivetrain(DcMotor rearRightMotor, DcMotor rearLeftMotor, DcMotor frontRightMotor, DcMotor frontLeftMotor, SparkFunOTOS otosSensor) {
        this.rearRightMotor = rearRightMotor;
        this.rearLeftMotor = rearLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.frontLeftMotor = frontLeftMotor;
        this.otosSensor = otosSensor;
    }

    public void move(double drive, double strafe, double turn) {
        frontLeftMotor.setPower(Range.clip(drive - strafe + turn, -1, 1));
        frontRightMotor.setPower(Range.clip(drive - strafe - turn, -1, 1));
        rearLeftMotor.setPower(Range.clip(drive + strafe + turn, -1, 1));
        rearRightMotor.setPower(Range.clip(drive + strafe - turn, -1, 1));

    }

}
