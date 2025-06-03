package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;

@TeleOp(name="NewOpMode", group="Iterative OpModes")
public class NewOpMode extends OpMode {
    Drivetrain scuba = new Drivetrain();

    public void init(){
        this.scuba = new Drivetrain();

    }

    public void loop(){
        scuba.driveTrainControl(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
