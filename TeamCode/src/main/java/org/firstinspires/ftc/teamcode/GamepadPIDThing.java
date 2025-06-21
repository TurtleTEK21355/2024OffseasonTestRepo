

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
    //enum Mode {KP, KI, KD}
    //Mode[] mode = new Mode[]{};


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

        int mode = 0;
        State kpState = State.SCREW_YOU;
        State kiState = State.SCREW_YOU;
        State kdState = State.SCREW_YOU;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;

        while (opModeIsActive() && !gamepad1.start) {
            if (gamepad1.dpad_left && !dpadLeftPressed){
                mode += 1;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left){
                dpadLeftPressed = false;
            }
            if (gamepad1.dpad_right && !dpadRightPressed){
                mode -= 1;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right){
                dpadRightPressed = false;
            }
            if (mode > 2){
                mode = 0;
            }
            if (mode < 0){
                mode = 2;
            }
            if (mode == 0) {
                if (gamepad1.dpad_up) {
                    kpState = State.UP;
                } else if (gamepad1.dpad_down) {
                    kpState = State.DOWN;
                }
            }
            if (mode == 1){
                if (gamepad1.dpad_up) {
                    kiState = State.UP;
                } else if (gamepad1.dpad_down) {
                    kiState = State.DOWN;
                }
            }
            if (mode == 2){
                if (gamepad1.dpad_up) {
                    kdState = State.UP;
                } else if (gamepad1.dpad_down) {
                    kdState = State.DOWN;
                }
            }
            if (elapsedTime.milliseconds() > 300){
                if (kpState == State.UP){
                    Kp += 0.01;
                } else if (kpState == State.DOWN){
                    Kp -= 0.01;
                }

                if (kiState == State.UP){
                    Ki += 0.01;
                } else if (kiState == State.DOWN){
                    Ki -= 0.01;
                }

                if (kdState == State.UP) {
                    Kd += 0.01;
                } else if (kdState == State.DOWN){
                    Kd -= 0.01;
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

            telemetry.addData("Mode(0=Kp, 1=Ki, 2=Kd)", mode);
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