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

    SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor leftViperSlide = null;
    private DcMotor rightViperSlide = null;
    private Servo grabberServo = null;
    private Servo grabberWristServo = null;
    private CRServo linearActuatorServo = null;
    private final double MOTOR = 751.8;
    private final double BottomLimit = 0.22;
    private final double TopLimit = 8.1;
    private final double viperSlideLimitBottom = MOTOR*BottomLimit;
    private final double viperSlideLimitTop = MOTOR*TopLimit;
    boolean field_centric = true;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void init() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        leftViperSlide = hardwareMap.get(DcMotor.class, "left_viper_slide");
        rightViperSlide = hardwareMap.get(DcMotor.class, "right_viper_slide");
        grabberServo = hardwareMap.get(Servo.class, "grabber_servo");
        grabberWristServo = hardwareMap.get(Servo.class, "grabber_hinge_servo");
        linearActuatorServo = hardwareMap.get(CRServo.class, "linear_actuator_servo");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        linearActuatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*rightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        leftViperSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightViperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        configureOtos();
    }

    @Override
    public void loop() {
        otos_update();
        encoder();
        move_robot();
        move_viper_slide_and_presets();
        move_grabber();
        move_grabber_wrist();
        move_linear_actuator();
        telemetry.update();
    }

    private void otos_update(){
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



    private void move_robot(){
        if (gamepad1.a){
            field_centric = !(field_centric);
        }
        if (field_centric){
            telemetry.addLine("Field Centric Driving ON");
            double y = -gamepad1.left_stick_y * 0.7;
            double x = gamepad1.left_stick_x * 0.7;
            double r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            double theta = Math.atan2(y,x);
            double correctedTheta = theta - myOtos.getPosition().h;
            double drive = r * Math.sin(correctedTheta);
            double strafe = r * Math.cos(correctedTheta);
            double turn = gamepad1.right_stick_x * 0.7;

            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

            frontLeftDrive.setPower(frontLeftStrafe);
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
        }
        else{
            telemetry.addLine("Field Centric Driving OFF");
            float drive = gamepad1.left_stick_y * -0.7f;
            float turn = gamepad1.right_stick_x * 0.7f;
            float strafe = gamepad1.left_stick_x * 0.7f;

            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

            frontLeftDrive.setPower(frontLeftStrafe);
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
        }
    }

    private void encoder() {
        telemetry.addData("leftviperslidelevel",leftViperSlide.getCurrentPosition());
        telemetry.addData("rightviperslidelevel",leftViperSlide.getCurrentPosition());
}

    private void move_viper_slide_and_presets() {
        /*leftViperSlide.setPower(gamepad2.left_stick_y);
        rightViperSlide.setPower(gamepad2.left_stick_y);


        if (leftViperSlide.getCurrentPosition()<0);
        if ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0<viperSlideLimitBottom){
            leftViperSlide.setPower(0.5);
            rightViperSlide.setPower(0.5);
        }
        else if ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0<viperSlideLimitTop){
            leftViperSlide.setPower((-gamepad2.left_stick_y)+idlePower);
            rightViperSlide.setPower((-gamepad2.left_stick_y)+idlePower);
        }
        else if (gamepad2.left_stick_y <= 0.1) {
            leftViperSlide.setPower(idlePower);
            rightViperSlide.setPower(idlePower);
        }
        else{
            leftViperSlide.setPower(0);
            rightViperSlide.setPower(0);
        }*/
        if (gamepad2.dpad_up){
            grabberWristServo.setPosition(1);
            move_preset(1);
        }
        else if (gamepad2.dpad_down) {
            grabberWristServo.setPosition(0.3);
            move_preset(2);
        }
        else{
            move_preset(0);
        }

    }

    private void move_preset(int viperPreset) {
        if (viperPreset == 1){
            double idlePower = 0.1;
            double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0);
            double viperSlidePower = 1;

            if (viperSlideEncoderAverage < viperSlideLimitTop){
                leftViperSlide.setPower(viperSlidePower);
                rightViperSlide.setPower(viperSlidePower);
            }
            else{
                leftViperSlide.setPower(idlePower);
                rightViperSlide.setPower(idlePower);
            }

        }
        else if (viperPreset == 2){
            double idlePower = 0.1;
            double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0);
            double viperSlidePower = -1;

            if (viperSlideLimitBottom < viperSlideEncoderAverage){
                leftViperSlide.setPower((viperSlidePower)+idlePower);
                rightViperSlide.setPower((viperSlidePower)+idlePower);
            }
            else{
                leftViperSlide.setPower(idlePower);
                rightViperSlide.setPower(idlePower);
            }
        }
        else if (viperPreset == 0){
            double idlePower = 0.1;
            double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0);
            double viperSlidePower = -gamepad2.left_stick_y;

            if (viperSlideLimitBottom < viperSlideEncoderAverage && viperSlideEncoderAverage < viperSlideLimitTop){
                leftViperSlide.setPower((viperSlidePower)+idlePower);
                rightViperSlide.setPower((viperSlidePower)+idlePower);
            }
            else if (viperSlideEncoderAverage > viperSlideLimitTop && viperSlidePower > -0.1){
                leftViperSlide.setPower(idlePower);
                rightViperSlide.setPower(idlePower);
            }
            else if (viperSlideEncoderAverage < viperSlideLimitBottom && 0.1 > viperSlidePower){
                leftViperSlide.setPower(idlePower);
                rightViperSlide.setPower(idlePower);
            }
            else{
                leftViperSlide.setPower((viperSlidePower)+idlePower);
                rightViperSlide.setPower((viperSlidePower)+idlePower);
            }
        }
    }

    private void move_viper_slide_manual() {
        double idlePower = 0.1;
        double viperSlideEncoderAverage = ((leftViperSlide.getCurrentPosition()+rightViperSlide.getCurrentPosition())/2.0);
        double viperSlidePower = -gamepad2.left_stick_y;

        if (viperSlideLimitBottom < viperSlideEncoderAverage && viperSlideEncoderAverage < viperSlideLimitTop){
            leftViperSlide.setPower((viperSlidePower)+idlePower);
            rightViperSlide.setPower((viperSlidePower)+idlePower);
        }
        else if (viperSlideEncoderAverage > viperSlideLimitTop && viperSlidePower > -0.1){
            leftViperSlide.setPower(idlePower);
            rightViperSlide.setPower(idlePower);
        }
        else if (viperSlideEncoderAverage < viperSlideLimitBottom && 0.1 > viperSlidePower){
            leftViperSlide.setPower(idlePower);
            rightViperSlide.setPower(idlePower);
        }
        else{
            leftViperSlide.setPower((viperSlidePower)+idlePower);
            rightViperSlide.setPower((viperSlidePower)+idlePower);
        }
    }

    private void move_grabber_wrist() {
        if (gamepad2.right_bumper) {
            grabberWristServo.setPosition(0.3);
        }
        else if (gamepad2.left_bumper) {
            grabberWristServo.setPosition(1);
        }

        if (gamepad2.dpad_right || gamepad2.dpad_left) {
            if (gamepad2.dpad_right) {
                grabberWristServo.setPosition(grabberWristServo.getPosition() + 0.05);
            }
            else if (gamepad2.dpad_left) {
                grabberWristServo.setPosition(grabberWristServo.getPosition() - 0.05);
            }
        }
    }

    private void move_grabber(){
        double open = 0.7;
        double close = 0.88;
        if(gamepad2.right_trigger>0.1){
            //close claw
            grabberServo.setPosition(close);
        }
        else if(gamepad2.left_trigger>0.1){
            //open claw
            grabberServo.setPosition(open);
        }

    }

    private void move_linear_actuator(){
        linearActuatorServo.setPower(gamepad2.right_stick_y);
    }

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
        myOtos.setLinearScalar(0.9961);
        myOtos.setAngularScalar(0.9899);

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
/*
public class Motor{
    protected double tickPerRotation;
    protected double tickPeriod;
    public abstract double getTicks();
}
public class Motor312 extends Motor{
    public Motor312(){
        this.tickPerRotation = 537.7;
    }
    public double getTicks(){
        return this.tickPerRotation;
    }
}
public class Wheel {
    protected double circumference;
    public abstract double getCircumference();
}
public class MeccanumWheel extends Motor{
    public MeccanumWheel(){
        this.circumference = 11.87373601357
    }
    public double getCircumference(){
        return this.circumference;
    }
}*/