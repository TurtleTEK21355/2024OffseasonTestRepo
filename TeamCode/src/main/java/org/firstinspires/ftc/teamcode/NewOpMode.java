package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="NewOpMode", group="Iterative OpModes")
public class NewOpMode extends OpMode {
    Robot scuba = new Robot();

    public void init(){
        this.scuba = new Robot();

    }

    public void loop(){
        scuba.driveTrainControl(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
