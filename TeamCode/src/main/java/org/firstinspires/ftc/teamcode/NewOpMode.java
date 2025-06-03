package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="NewOpMode", group="Iterative OpModes")
public class NewOpMode extends OpMode {
    public void init(){
        Scuba Scuba1 = new Scuba();
        Scuba1.driveTrainControl();
    }
    public void loop(){

    }
}
