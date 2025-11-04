package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.MeccanumWheeDrivetrain;
import org.firstinspires.ftc.teamcode.robot.Scuba2;
import org.firstinspires.ftc.teamcode.robot.Scuba2Claw;

@TeleOp(name="NewOpMode", group="Iterative OpModes")
public class NewTeleOp extends OpMode {
    Scuba2 scuba2;

    public void init(){
        scuba2 = new Scuba2(
            hardwareMap.get(Servo.class, "grabber_servo"),
            hardwareMap.get(Servo.class, "grabber_rotate_servo"),
            hardwareMap.get(Servo.class, "grabber_tilt_servo"),
            hardwareMap.get(DcMotor.class, "left_viper_slide"),
            hardwareMap.get(DcMotor.class, "right_viper_slide"),
            hardwareMap.get(TouchSensor.class, "viper_slide_touch"),
            hardwareMap.get(DcMotor.class, "linear_actuator_motor")
        );
        scuba2.drivetrain.addMotor(hardwareMap.get(DcMotor.class, "front_left_drive"), MeccanumWheeDrivetrain.WheelPosition.FRONT_LEFT);
        scuba2.drivetrain.addMotor(hardwareMap.get(DcMotor.class, "front_right_drive"), MeccanumWheeDrivetrain.WheelPosition.FRONT_RIGHT);
        scuba2.drivetrain.addMotor(hardwareMap.get(DcMotor.class, "rear_left_drive"), MeccanumWheeDrivetrain.WheelPosition.REAR_LEFT);
        scuba2.drivetrain.addMotor(hardwareMap.get(DcMotor.class, "rear_right_drive"), MeccanumWheeDrivetrain.WheelPosition.REAR_RIGHT);
        scuba2.drivetrain.addOtos(hardwareMap.get(SparkFunOTOS.class, "sensor_otos"));

    }

    public void loop(){
        //move robot
        scuba2.drivetrain.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //viper slides
        scuba2.arm.viperSlides.movePower(-gamepad2.left_stick_y);
        telemetry.addLine(scuba2.arm.viperSlides.getStats());

        //linear actuator
        scuba2.arm.linearActuator.move(-gamepad2.right_stick_y);

        //open and close claw
        if (gamepad2.right_trigger >= 0.1){
            scuba2.arm.claw.close();
        }
        else if(gamepad2.left_trigger >= 0.1){
            scuba2.arm.claw.open();
        }

        //rotate claw
        if (gamepad2.right_stick_x >= 0.1) {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.RIGHT);

        }
        else if (gamepad2.right_stick_x <= -0.1) {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.LEFT);
        }
        else {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.MIDDLE);
        }

        //tilt claw
        if (gamepad2.right_bumper){
            scuba2.arm.claw.tilt(Scuba2Claw.Tilt.UP);
        }
        else if(gamepad2.left_bumper){
            scuba2.arm.claw.tilt(Scuba2Claw.Tilt.DOWN);
        }
    }
}
