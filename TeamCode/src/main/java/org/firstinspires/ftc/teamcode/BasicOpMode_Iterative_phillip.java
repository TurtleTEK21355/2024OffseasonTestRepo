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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode Phillip", group="Iterative OpMode")
public class BasicOpMode_Iterative_phillip extends OpMode {

    SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor leftViperSlide = null;
    private DcMotor rightViperSlide = null;
    private Servo grabberServo = null;
    private Servo grabberHingeServo = null;
    private CRServo linearActuatorServo = null;
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
        grabberHingeServo = hardwareMap.get(Servo.class, "grabber_hinge_servo");
        linearActuatorServo = hardwareMap.get(CRServo.class, "linear_actuator_servo");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftViperSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightViperSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearActuatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        configureOtos();
    }

    @Override
    public void loop() {
        otos_update();
        move_robot();
        move_viper_slide();
        move_grabber();
        move_grabber_hinge();
        move_linear_actuator();
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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            double theta = Math.atan2(y,x);
            double correctedTheta = theta - myOtos.getPosition().h;
            double drive = r * Math.sin(correctedTheta);
            double strafe = r * Math.cos(correctedTheta);
            double turn = gamepad1.right_stick_x;

            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

            frontLeftDrive.setPower(frontLeftStrafe);
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
            telemetry.update();
        }
        else{
            telemetry.addLine("Field Centric Driving OFF");
            float drive = -gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x;
            float strafe = gamepad1.left_stick_x;

            double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

            frontLeftDrive.setPower(frontLeftStrafe);
            frontRightDrive.setPower(frontRightStrafe);
            rearLeftDrive.setPower(rearLeftStrafe);
            rearRightDrive.setPower(rearRightStrafe);
            telemetry.update();
        }
    }

    private void move_viper_slide() {
        leftViperSlide.setPower(gamepad2.left_stick_y);
        rightViperSlide.setPower(gamepad2.left_stick_y);
    }

    private void move_grabber_hinge() {
        if(gamepad2.dpad_down){
            grabberHingeServo.setPosition(0.3);
        }
        else if(gamepad2.dpad_up){
            grabberHingeServo.setPosition(1);
        }


    }

    private void move_grabber(){
        if(gamepad1.right_trigger>0.1){
            //close claw
            grabberServo.setPosition(0.9);
        }
        else if(gamepad1.left_trigger>0.1){
            //open claw
            grabberServo.setPosition(0.7);
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