package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MeccanumWheelRobot extends Robot {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    public void driveTrainControl(double drive, double strafe, double turn) {
        frontLeftMotor.setPower(Range.clip(drive - strafe + turn, -1, 1));
        frontRightMotor.setPower(Range.clip(drive - strafe - turn, -1, 1));
        rearLeftMotor.setPower(Range.clip(drive + strafe + turn, -1, 1));
        rearRightMotor.setPower(Range.clip(drive + strafe - turn, -1, 1));

    }
}
