

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="gamepadauto", group="Test OpMode")
public class GamepadPIDThing extends LinearOpMode {
    // Declare OpMode members.
    //SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private float frontLeftStrafe;
    private float frontRightStrafe;
    private float rearLeftStrafe;
    private float rearRightStrafe;
    private double Kp = 0.09;
    private double Ki = 0;
    private double Kd = 0;
    private double KpTheta = 0.09;
    private double KiTheta = 0;
    private double KdTheta = 0;
    enum State {UP, DOWN, SCREW_YOU}


    @Override
    public void runOpMode() {
        declareMotors();
        waitForStart();
        configurePID();

    }
    public void declareMotors(){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void configurePID(){
        ModeController modeController = new ModeController();
        modeController.add(
            new Mode(0, "Kp"),
            new Mode(0, "Ki"),
            new Mode(0, "Ki")
        );

        while(opModeIsActive() && !gamepad1.start) {
            modeController.modeSelection(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);

            telemetry.addLine("Press start to Start");

            if (gamepad1.dpad_down) {
                telemetry.addLine("Dpad Down");
            } else if (gamepad1.dpad_up) {
                telemetry.addLine("Dpad Up");
            } else {
                telemetry.addLine("");
            }

            telemetry.addData("Mode:", modeController.getModeName());
            telemetry.addLine(modeController.reportModeValue());
            telemetry.
            telemetry.update();

        }
    }
}