package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="DistanceTester",group = "Linear OpMode")

public class DistanceCallibrationTest extends LinearOpMode {
    SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private float frontLeftStrafe;
    private float frontRightStrafe;
    private float rearLeftStrafe;
    private float rearRightStrafe;


    public void runOpMode() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        SparkFunOTOS.Pose2D pos;
        myOtos.resetTracking();
        pos = myOtos.getPosition();

        while (pos.y < 75 && opModeIsActive()) {
            drivetrainControl(0.3f,  0, (float)myOtos.getPosition().h*(float)0.01);
            pos = myOtos.getPosition();
            telemetry.addData("Y coordinate", pos.y);
            telemetry.update();
        }
        drivetrainControl(0, 0, 0);

//        while(pos.x > -50 &&opModeIsActive()) {
//            pos = myOtos.getPosition();
//            drivetrainControl(0, -0.3f, (float)myOtos.getPosition().h*(float)0.02);
//            telemetry.addData("X coordinate", pos.x);
//            telemetry.update();
//        }
//        drivetrainControl(0,0,0);
}



    void drivetrainControl(float drive, float strafe, float turn) {
        frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
        frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
        rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
        rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

        frontLeftDrive.setPower(frontLeftStrafe);
        frontRightDrive.setPower(frontRightStrafe);
        rearLeftDrive.setPower(rearLeftStrafe);
        rearRightDrive.setPower(rearRightStrafe);
    }
    public void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.9961);
        myOtos.setAngularScalar(0.9889);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }
}



