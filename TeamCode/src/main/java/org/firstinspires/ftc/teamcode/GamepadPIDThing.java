

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
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
        State kpState = State.SCREW_YOU;
        State kiState = State.SCREW_YOU;
        State kdState = State.SCREW_YOU;
        while (opModeIsActive() && !gamepad1.a) {
            if (gamepad1.dpad_up) {
                kpState = State.UP;
            } else if (gamepad1.dpad_down) {
                kpState = State.DOWN;
            }
            if (-gamepad1.left_stick_y > 0.01) {
                kiState = State.UP;
            } else if (-gamepad1.left_stick_y < -0.01) {
                kiState = State.DOWN;
            }
            if (-gamepad1.right_stick_y > 0.01) {
                kdState = State.UP;
            } else if (-gamepad1.right_stick_y < -0.01) {
                kdState = State.DOWN;
            }
            if (elapsedTime.milliseconds() > 100){
                if (kpState == State.UP){
                    Kp += 0.01;
                } else if (kpState == State.DOWN){
                    Kp -= 0.01;
                }
//                switch(kpState){
//                    case UP: Kp += 0.01;
//                    case DOWN: Kp -= 0.01;
//                    case SCREW_YOU:
//                }
                switch(kiState){
                    case UP: Ki += 0.01;
                    case DOWN: Ki -= 0.01;
                }
                switch(kdState){
                    case UP: Kd += 0.01;
                    case DOWN: Kd -= 0.01;
                    default:
                }
                kpState = State.SCREW_YOU;
                kiState = State.SCREW_YOU;
                kdState = State.SCREW_YOU;
                elapsedTime.reset();
            }
            telemetry.addLine("Press A to Start");
            if (gamepad1.dpad_down){
                telemetry.addLine("Dpad Down");
            }
            else if (gamepad1.dpad_up){
                telemetry.addLine("Dpad Up");
            }
            else {
                telemetry.addLine("");
            }
            telemetry.addData("leftStick:", -gamepad1.left_stick_y);
            telemetry.addData("rightStick", -gamepad1.right_stick_y);
            telemetry.addData("KpState:", kpState.name());
            telemetry.addData("KiState:", kiState.name());
            telemetry.addData("KdState:", kdState.name());
            telemetry.addData("Kp:", Kp);
            telemetry.addData("Ki:", Ki);
            telemetry.addData("Kd:", Kd);
            telemetry.update();
        }
    }
}