package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MeccanumWheeDrivetrain;
import org.firstinspires.ftc.teamcode.robot.Scuba2;
import org.firstinspires.ftc.teamcode.robot.Scuba2Claw;

@TeleOp(name="NewOpMode", group="Iterative OpModes")
public class NewOpMode extends OpMode {
    Scuba2 scuba2;

    public void init(){
        scuba2 = new Scuba2(
            hardwareMap.get(DcMotor.class, "front_left_drive"),
            hardwareMap.get(DcMotor.class, "front_right_drive"),
            hardwareMap.get(DcMotor.class, "rear_left_drive"),
            hardwareMap.get(DcMotor.class, "rear_right_drive"),
            hardwareMap.get(Servo.class, "grabber_servo"),
            hardwareMap.get(Servo.class, "grabber_rotate_servo"),
            hardwareMap.get(Servo.class, "grabber_tilt_servo"),
            hardwareMap.get(DcMotor.class, "left_viper_slide"),
            hardwareMap.get(DcMotor.class, "right_viper_slide"),
            hardwareMap.get(DcMotor.class, "linear_actuator_motor")
        );


    }

    public void loop(){
        scuba2.drivetrain.move(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad2.right_stick_x >= 0.1) {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.RIGHT);

        }
        else if (gamepad2.right_stick_x <= -0.1) {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.LEFT);
        }
        else {
            scuba2.arm.claw.rotate(Scuba2Claw.Rotation.MIDDLE);
        }

    }
}
