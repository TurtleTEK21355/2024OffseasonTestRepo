/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode {


    SparkFunOTOS myOtos; //add to field centric driving class
    private DcMotor frontLeftDrive = null;  //move drives to class
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor leftViperSlide = null;  //move viper slides to class
    private DcMotor rightViperSlide = null;
    private Servo grabberServo = null;   // move grabber to class
    private Servo grabberRotateServo = null;
    private Servo grabberTiltServo = null;
    private CRServo linearActuatorServo = null; // change to motor, 
    private final double MOTOR = 751.8; //make variable more descriptive - say it is for the viper slide  (add to viper slide class)
    // consider making these not constants, but presets that the operator can modify with button presses if they are not working
    private final double BottomLimit = 0.22; // make variable more descritive - say it is for the viper slide (add to viper slide class)
    private final double TopLimit = 8.1;  // make variable more descritive - say it is for the viper slide (add to viper slide class)
    //add feature to disable limits
    private final double viperSlideLimitBottom = MOTOR*BottomLimit; //add to viper slide class
    private final double viperSlideLimitTop = MOTOR*TopLimit; //add to viper slide class
    private boolean field_centric = true; //document that the program defaults to field centric being on
    private int lastViperPreset = 0; //add to viper slide class as an enum
    /*
     public enum ViperSlidePreset {
        UP,
        DOWN,
        MANUAL;
    }    
     */

    SparkFunOTOS.Pose2D pos; //add to field centric driving class 

    @Override
    public void init() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); // add to field centric driving class 
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive"); // add to drive class
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive"); // add to drive class
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive"); // add to drive class
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive"); // add to drive class
        leftViperSlide = hardwareMap.get(DcMotor.class, "left_viper_slide"); // add to viper slide class 
        rightViperSlide = hardwareMap.get(DcMotor.class, "right_viper_slide"); // add to viper slide class 
        grabberServo = hardwareMap.get(Servo.class, "grabber_servo"); //add to grabber class 
        grabberRotateServo = hardwareMap.get(Servo.class, "grabber_rotate_servo"); // add to grabber class 
        grabberTiltServo = hardwareMap.get(Servo.class, "grabber_tilt_servo"); // add to grabber class
        linearActuatorServo = hardwareMap.get(CRServo.class, "linear_actuator_servo"); // add to linear actuator class

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE); //add to drive class
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD); //add to drive class
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE); //add to drive class
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD); //add to drive class
        linearActuatorServo.setDirection(DcMotorSimple.Direction.REVERSE);  // add to linera actuator class 
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to drive class 
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to drive class 
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to drive class 
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to drive class 
        //delete commented out code or explain why it is commented out
        /*rightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        leftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        leftViperSlide.setDirection(DcMotorSimple.Direction.FORWARD); // add to viper slide class
        rightViperSlide.setDirection(DcMotorSimple.Direction.REVERSE); // add to viper slide class
        rightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to viper slide class
        leftViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // add to viper slide class

        configureOtos(); // add to field centric driving class
    }

    @Override
    public void loop() {
        //drive.move()
        otos_update(); // do in field centric driving class
        encoder(); // move to viper slide class (this function is too generically named)
        move_robot(); //move to drive class 
        //viperslide.move()
        move_viper_slide_and_presets(); // move to viper slide class
        //write.move()
        move_grabber(); // move to grabber class
        move_grabber_wrist(); //move to grabber class 
        //linear_actuator.move()
        move_linear_actuator(); //move to linear_actuator class
        telemetry.update();
    }

    private void otos_update(){  //move to field centric driving class
        pos = myOtos.getPosition();
        // Reset the tracking if the user requests it
        if (gamepad1.y) {
            myOtos.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (gamepad1.x) {
            myOtos.calibrateImu();
        }


        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle" , pos.h);

    }



    private void move_robot(){ //move to drive class  Scuba2Drive.move()
        if (gamepad1.a){ //use getter property for "getFieldCentricDrivingToggleButtonPressed()"
            field_centric = !(field_centric);  // move to loop - re-init driving class when pushed (field centric vs regular)
        }
        if (field_centric){
            //move to field centric driving class in
            telemetry.addLine("Field Centric Driving ON"); 
            
            //move next  8 lines to FieldCentricDriving.getDriveStrafeTurn() -> (drive, strafe, turn)
            double y = -gamepad1.left_stick_y * 0.7;  //use getter property for "getDrivingForwardPower()" 
            double x = gamepad1.left_stick_x * 0.7; // use getter property for "getDrivingSidewaysPower()"
            double r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2)); 
            double theta = Math.atan2(y,x);
            double correctedTheta = theta - myOtos.getPosition().h;
            double drive = r * Math.sin(correctedTheta);
            double strafe = r * Math.cos(correctedTheta);
            double turn = gamepad1.right_stick_x * 0.7; // user getter property for "getDrivingTurnPower()"
            //move next 4 lines to Driving.getDrivePowers(drive, strafe, turn)->(frontLeftPower,frontRightPower,rearLeftPower,readRightPower)
            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1); 
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);
            //move next 4 lines to Driving.setDrivePowers(frontLeftPower,frontRightPower,rearLeftPower,readRightPower)
            frontLeftDrive.setPower(frontLeftStrafe); 
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
        }
        else{
            telemetry.addLine("Field Centric Driving OFF");
            //move next 3 lines to Driving.getDriveStrafeTurn() -> (drive, strafe, turn)
            float drive = gamepad1.left_stick_y * -0.7f; //why is this float and the field centric driving is not?
            float turn = gamepad1.right_stick_x * 0.7f;
            float strafe = gamepad1.left_stick_x * 0.7f;
            //reuse Driving.getDrivePowers from above
            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);
            //resue Driving.setDrivePowers from above
            frontLeftDrive.setPower(frontLeftStrafe);
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
        }
    }
    
    private void encoder() { //move to viper slide class (and rename this function )
        telemetry.addData("leftviperslidelevel",leftViperSlide.getCurrentPosition()); //most of your telemetry data has better formatted text
        telemetry.addData("rightviperslidelevel",leftViperSlide.getCurrentPosition());
    }

    private void move_viper_slide_and_presets() { //move to viper slide class
        //create a method to bring both slides all the way down and reset the encoders
        if (gamepad2.dpad_up){ // move to ViperSlide.getViperSlideUpPressed() 
            grabberTiltServo.setPosition(1); // move to the grabber class 
            lastViperPreset = 1; // use enum for viper preset 
        }
        else if (gamepad2.dpad_down) { // move to ViperSlide.getViperSlideDownPressed()
            //create a method for going all the way down 
            grabberTiltServo.setPosition(0.3);  // move to grabber class 
            lastViperPreset = 2; // use enum for viper preset
        }
        else if (Math.abs(gamepad2.left_stick_y)>0.05){  // move to ViperSlide.getViperSlidePresetOverride()
            lastViperPreset = 0; // use enum for viper preset 
        }
        move_preset(lastViperPreset); //move to ViperSlide.move() function 

    }

    private void move_preset(int viperPreset) {
        //function should be split into ViperSlide.getViperSlidePower() and ViperSlide.setViperSlidePower()
        if (viperPreset == 1){ // use enum for viper preset
            // move to ViperSlide.moveToUpLimit() function
            double idlePower = 0.1;  //move to constant on ViperSlide Class
            // report viper slide current position in telemetery, driver should be able to reset encoders if it seems off
            // if viper slide positions are off by X, then inform driver that it should be reset 
            double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0);   //move to ViperSlide.getViperSlidePosition()
            double viperSlidePower = 1; // move to constant on ViperSlide class ViperSlide.UP_POWER

            if (viperSlideEncoderAverage < viperSlideLimitTop){ //ViperSlide.LIMIT_TOP (use constant on the ViperSlide class)
                //should return viperSlidePower here
                leftViperSlide.setPower(viperSlidePower); //move to ViperSlide.setViperSlidePower()
                rightViperSlide.setPower(viperSlidePower); //move to ViperSlide.setViperSlidePower()
            }
            else{
                //should return viperSlidePower here
                leftViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
                rightViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
            }

        }
        else if (viperPreset == 2){ //use enum for viper preset
            // move to ViperSlide.moveToDownLimit() function
            double idlePower = 0.1; //use constant on ViperSlideClass "private static final ViperSlide.IDLE_POWER"
            // report viper slide current position in telemetery, driver should be able to reset encoders if it seems off
            // if viper slide positions are off by X, then inform driver that it should be reset 
            double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0); //move to ViperSlide.getViperSlidePosition()
            double viperSlidePower = -1; //use constant  ViperSlide.DOWN_POWER

            if (viperSlideLimitBottom < viperSlideEncoderAverage){ // ViperSlide.LIMIT_BOTTOM (use constant)
                leftViperSlide.setPower((viperSlidePower)+idlePower); // move to ViperSlide.setViperSlidePower()
                rightViperSlide.setPower((viperSlidePower)+idlePower); // move to ViperSlide.setViperSlidePower()
            }
            else{
                leftViperSlide.setPower(idlePower); // move to ViperSlide.setViperSlidePower()
                rightViperSlide.setPower(idlePower); // move to ViperSlide.setViperSlidePower()
            }
        }
        else if (viperPreset == 0){ // use enum for constant            
            move_viper_slide_manual();  // move to ViperSlide.moveManual() function
        }
    }
    
    private void move_viper_slide_manual() { // move to ViperSlide.moveManual() function
        
        double idlePower = 0.1; // use constant  from ViperSlide Class 
        // report viper slide current position in telemetery, driver should be able to reset encoders if it seems off
        // if viper slide positions are off by X, then inform driver that it should be reset 
        double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0); //move to ViperSlide.getViperSlidePosition()        
        // Question: should this simply be +1 or -1 depending on the if the stick is up or down?  
        double viperSlidePower = -gamepad2.left_stick_y; //get value from ViperSlide.getManualViperSlideOperatorPower() property - explain why power is negative

        //move this logic to its own function ViperSlide.getManualViperSlidePower() property, it should be tested 
        if (viperSlideLimitBottom < viperSlideEncoderAverage && viperSlideEncoderAverage < viperSlideLimitTop){ 
            leftViperSlide.setPower((viperSlidePower)+idlePower); //move to ViperSlide.setViperSlidePower()
            rightViperSlide.setPower((viperSlidePower)+idlePower); //move to ViperSlide.setViperSlidePower()
        }
        else if (viperSlideEncoderAverage > viperSlideLimitTop && viperSlidePower > -0.1){ 
            // Question: should the tilt servo move up here
            leftViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
            rightViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
        }
        else if (viperSlideEncoderAverage < viperSlideLimitBottom && 0.1 > viperSlidePower){ 
            // Question: should the tilt servo move down here?
            leftViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
            rightViperSlide.setPower(idlePower); //move to ViperSlide.setViperSlidePower()
        }
        else{
            leftViperSlide.setPower((viperSlidePower)+idlePower); //move to ViperSlide.setViperSlidePower()
            rightViperSlide.setPower((viperSlidePower)+idlePower); //move to ViperSlide.setViperSlidePower()
        }
        
    }
    //the Scuba2Grabber is composed of "Tilter, Rotater and Claw: these could be distinct classes"
    private void move_grabber_tilt() { //move to grabber class Grabber.moveTilt(ViperSlidePreset)
        //Take viper slide preset into account here so we don't conflict with previous move
        //there are two distinct activities happening in this function (rotate + tilt, should be split up) 
        if (gamepad2.right_bumper) { //move to  Grabber.isOperatorGrabberTiltUpPressed() 
            grabberTiltServo.setPosition(0.3); // use constants: Grabber.TILT_DOWN_SERVO_POSITION = 0.3
        }
        else if (gamepad2.left_bumper) { //move to  Grabber.isOperatorGrabberTiltDownPressed() 
            grabberTiltServo.setPosition(1); // use constants Grabber.TILT_UP_SERVO_POSITION = 1
        }

        if (gamepad2.dpad_right || gamepad2.dpad_left) { // remove unnecessary statement
            if (gamepad2.dpad_right) { // move to Grabber.isOperatorGrabberRotateRightPressed()
                grabberRotateServo.setPosition(grabberRotateServo.getPosition() + 0.05); //move to Grabber.rotateRight()
            }
            else if (gamepad2.dpad_left) { // move to Grabber.isOperatorGrabberRotateLeftPressed()
                grabberRotateServo.setPosition(grabberRotateServo.getPosition() - 0.05); // move to Grabber.rotateLeft()
            }
        }
    }

    private void move_grabber(){ //move to Grabber.moveClaw()
        double open = 0.7; // move to constants Grabber.CLAW_OPEN_SERVO_POSITION
        double close = 0.88; // move to constants Grabber.CLAW_CLOSE_SERVO_POSITION
        if(gamepad2.right_trigger>0.1){ // move to Grabber.isOperatorGrabberClawClosePressed()
            //close claw
            grabberServo.setPosition(close); //move to Grabber.close()
        }
        else if(gamepad2.left_trigger>0.1){ // move to Grabber.isOperatorGrabberClawOpenPressed()
            //open claw
            grabberServo.setPosition(open); // move to Grabber.Open()
        }

    }
    // move to LinearActuator (or arm) class, this hould be arm.move()
    private void move_linear_actuator(){
        //TODO: change to motor functions
        //Question: can you call it "arm" instead of LinearActuator? 
        linearActuatorServo.setPower(gamepad2.right_stick_y); // split into LinearActuator.getPower and LinearActuator.SetPower
    }
    // move to Scuba2OdometrySensor class
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(0.9961); //add constants here 
        myOtos.setAngularScalar(0.9899); //add constants here 

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}