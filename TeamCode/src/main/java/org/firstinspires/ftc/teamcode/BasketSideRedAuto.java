/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="BasketSideAuto", group="Linear OpMode")
public class BasketSideRedAuto extends LinearOpMode {
    // Declare OpMode members.
    ElapsedTime elapsedTime;
    SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;
    private DcMotor leftViperSlide;
    private DcMotor rightViperSlide;
    private Servo grabberServo;
    private Servo grabberHingeServo;
    private CRServo linearActuatorServo;


    @Override
    public void runOpMode() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        elapsedTime = new ElapsedTime();
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
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberServo.setPosition(0.5);
        configureOtos();
        waitForStart();



        SparkFunOTOS.Pose2D pos;
        myOtos.resetTracking();
        pos = myOtos.getPosition();
        // Drive away from the wall
        while (elapsedTime.seconds() < 0.5){
            leftViperSlide.setPower(-0.2);
            rightViperSlide.setPower(-0.2);
        }
        grabberHingeServo.setPosition(0.35);
        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            leftViperSlide.setPower(0.2);
            rightViperSlide.setPower(0.2);
        }
        while (pos.y < 4 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        // Strafe to be in line with the farthest spike mark from the wall
        while (pos.x > -10 && opModeIsActive()) {
            drivetrainControl(0, -0.3f, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        // Drive towards the spike mark
        while (pos.y < 15.5 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        elapsedTime.reset();
        //Grabby Grabby
        while(elapsedTime.seconds() < 4 && opModeIsActive()){
            linearActuatorServo.setPower(1);
        }
        linearActuatorServo.setPower(0);
        sleep(500);
        sampleIntake();
        sleep(500);
        // Drive away from the spike mark
        while (pos.y > -9 && opModeIsActive()) {
            drivetrainControl(-0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();

        }

        stopAllMotors();
        myOtos.resetTracking();
        //Turn towards the basket
        while (pos.h < 115 && opModeIsActive()) {
            drivetrainControl(0, 0, -0.3f); //unsure as to turning direction
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        //Drive towards the basket
        while (pos.y < 6 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
        }
        stopAllMotors();
        myOtos.resetTracking();
        myOtos.calibrateImu();
        elapsedTime.reset();
        basketScore();
        myOtos.resetTracking();
        // Turn away from the baskets, towards the second spike mark
        while (pos.h > -125 && opModeIsActive()) {
            drivetrainControl(0, 0, 0.3f);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        myOtos.resetTracking();
        // Drive away from the baskets
        while (pos.y < 8 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        myOtos.calibrateImu();
        while (pos.x > -2 && opModeIsActive()) {
            drivetrainControl(0, -0.3f, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
        }
        elapsedTime.reset();
        sleep(500);
        sampleIntake();
        sleep(500);
        while (pos.y < 7 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        myOtos.calibrateImu();
        while (pos.y > -12 && opModeIsActive()) {
            drivetrainControl(-0.3f, 0, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        myOtos.calibrateImu();
        //Turn towards basket
        while (pos.h < 135 && opModeIsActive()) {
            drivetrainControl(0, 0, -0.3f);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();
        myOtos.resetTracking();
        myOtos.calibrateImu();
        elapsedTime.reset();
        basketScore();
        //Drive to the ascent zone
        while (pos.y > -25 && opModeIsActive()) {
            drivetrainControl(-0.3f, 0f, 0);
            pos = myOtos.getPosition();
            telemetry.addData("X coord", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading", pos.h);
            telemetry.update();
        }
        stopAllMotors();

    }
    private void stopAllMotors(){
        drivetrainControl(0,0,0);
    }
    private void driveStraight(float power, float position){
        float turn = position/100;
        drivetrainControl(power,0,turn);
    }
    private void driveStrafe(float power, float position){
        float turn = position/100;
        drivetrainControl(0,power,turn);
    }

    private void drivetrainControl(float drive, float strafe, float turn) {
        double frontLeftStrafe = Range.clip(drive + strafe + turn, -1, 1);
        double frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
        double rearLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
        double rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);

        frontLeftDrive.setPower(frontLeftStrafe);
        frontRightDrive.setPower(frontRightStrafe);
        rearLeftDrive.setPower(rearLeftStrafe);
        rearRightDrive.setPower(rearRightStrafe);
    }
    private void sampleIntake(){

        grabberServo.setPosition(0.93);
    }

    private void basketScore(){
        while (elapsedTime.seconds() < 2.35 && opModeIsActive()) {
            leftViperSlide.setPower(-0.78);
            rightViperSlide.setPower(-0.78);
        }
        myOtos.resetTracking();
        while (myOtos.getPosition().y < 1.5 && elapsedTime.seconds() > 1.25 && opModeIsActive()) {
            drivetrainControl(0.3f, 0, 0);
        }
        leftViperSlide.setPower(0);
        rightViperSlide.setPower(0);
        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5 && opModeIsActive()) {
            grabberServo.setPosition(0.5);
        }
        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5 && opModeIsActive()) {
            drivetrainControl(-0.3f,0,0);
        }
        stopAllMotors();
        while (elapsedTime.seconds() < 2 && opModeIsActive()) {
            leftViperSlide.setPower(0.75);
            rightViperSlide.setPower(0.75);
        }
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
        myOtos.setAngularUnit(AngleUnit.DEGREES);

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
