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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

@Autonomous(name="PIDBasketAuto", group="Test OpMode")
public class PIDAutoTest extends LinearOpMode {
    // Declare OpMode members.
    SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor leftViperSlide = null;
    private DcMotor rightViperSlide = null;
    private Servo grabberServo = null;
    private Servo grabberHingeServo = null;
    private DcMotor linearActuatorMotor = null;
    private float frontLeftStrafe;
    private float frontRightStrafe;
    private float rearLeftStrafe;
    private float rearRightStrafe;

    private double Kp = 0.1;
    private double Ki = 0;
    private double Kd = 0;
    private double KpTheta = 0.1;
    private double KiTheta = 0;
    private double KdTheta = 0;
    private int[] scoringPos = new int[]{10,0,-90}; //Filler Values
    private int[] spikeMark1 = new int[]{5,-3,0}; // Test Value
    private int[] spikeMark2 = new int[]{15,0,90}; // Place-holder values`
    private int[] spikeMark3 = new int[]{0,0,0};
    private int[] parkingPos = new int[]{0,0,0};





    @Override
    public void runOpMode() {

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        leftViperSlide = hardwareMap.get(DcMotor.class, "left_viper_slide");
        rightViperSlide = hardwareMap.get(DcMotor.class, "right_viper_slide");
        grabberServo = hardwareMap.get(Servo.class, "grabber_servo");
        grabberHingeServo = hardwareMap.get(Servo.class, "grabber_tilt_servo");
        linearActuatorMotor = hardwareMap.get(DcMotor.class, "linear_actuator_motor");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        configureOtos();
        grabberServo.setPosition(0.9);
        waitForStart();
        SparkFunOTOS.Pose2D pos;
        pos = myOtos.getPosition();
        /*while (leftViperSlide.getCurrentPosition() > -100){
            viperControl(-0.5);
        }
        sleep(1000);
        grabberHingeServo.setPosition(0.8);
        stopAllMotors();
        telemetry.addData("ArmPos",leftViperSlide.getCurrentPosition());
        telemetry.update();
        while (leftViperSlide.getCurrentPosition() > -5250) {
            telemetry.addData("ArmPos",leftViperSlide.getCurrentPosition());
            telemetry.update();
            viperControl(-0.8);
        }
        stopAllMotors();
         */
        positionControlWithTheta(scoringPos[0],scoringPos[1],scoringPos[2],0.5f,0,0.5f);
        /*stopAllMotors();
        viperControl(-0.05);
        sleep(100);
        grabberServo.setPosition(0.2);
        sleep(1000);
        stopAllMotors();
        while (leftViperSlide.getCurrentPosition() < -2000) {
            telemetry.addData("ArmPos",leftViperSlide.getCurrentPosition());
            telemetry.update();
            viperControl(0.7);
        }
        grabberHingeServo.setPosition(0.3);
         */
        sleep(9000);
        positionControlWithTheta(spikeMark1[0], spikeMark1[1],spikeMark1[2],0.5f,0.5f,0.5f);
        /*
        sleep(1000);
        grabberServo.setPosition(0.9);
        sleep(500);
        stopAllMotors();
        while (leftViperSlide.getCurrentPosition() > -5250) {
            telemetry.addData("ArmPos",leftViperSlide.getCurrentPosition());
            telemetry.update();
            viperControl(-0.8);
        }
         */
        sleep(9000);
        positionControlWithTheta(scoringPos[0],scoringPos[1],scoringPos[2],0.5f,0.5f,0.5f);
        /*
        viperControl(-0.05);
        sleep(100);
        grabberServo.setPosition(0.2);
        sleep(1000);
        stopAllMotors();
         */
        sleep(9000);
        positionControlWithTheta(spikeMark2[0], spikeMark2[1], spikeMark2[2], 0.5f,0.5f,0.5f);
        /*
        sleep(200);
        grabberHingeServo.setPosition(0.3);
        while (leftViperSlide.getCurrentPosition() < -545) {
            telemetry.addData("ArmPos",leftViperSlide.getCurrentPosition());
            telemetry.update();
            viperControl(1);
        }
*/
        stopAllMotors();
    }
    private void positionControl(float targetYPos, float targetXPos, float MaxYSpeed, float MaxXSpeed) {
        double previousErrorY = 0, previousErrorX = 0;
        double integralY = 0, integralX = 0;
        double speedY = MaxYSpeed, speedX = MaxXSpeed;

        while (true) {
            double currentY = myOtos.getPosition().y;
            double currentX = myOtos.getPosition().x;

            double errorY = targetYPos - currentY;
            double errorX = targetXPos - currentX;

            if (Math.abs(errorY) <= 0.1 && Math.abs(errorX) <= 0.1) {
                break;
            }

            integralY = integralY + errorY;
            integralX = integralX + errorX;

            double derivativeY = errorY - previousErrorY;
            double derivativeX = errorX - previousErrorX;

            double yPower = Math.min((Kp * errorY) + (Ki * integralY) + (Kd * derivativeY), Math.abs(speedY));
            double xPower = Math.min((Kp * errorX) + (Ki * integralX) + (Kd * derivativeX), Math.abs(speedX));

            previousErrorY = errorY;
            previousErrorX = errorX;
            telemetry.addData("YPos", myOtos.getPosition().y);
            telemetry.addData("XPos", myOtos.getPosition().x);
            telemetry.addData("ThetaPos", myOtos.getPosition().h);
            telemetry.update();
            // Send calculated power to drivetrain
            drivetrainControl((float) yPower, (float) xPower, 0);
        }

        stopAllMotors(); // Stop the robot once the target is reached
    }

    private void positionControlWithTheta(float targetYPos, float targetXPos, float targetThetaPos, float MaxYSpeed, float MaxXSpeed, float MaxThetaSpeed) {
        double previousErrorY = 0, previousErrorX = 0, previousErrorTheta = 0;
        double integralY = 0, integralX = 0, integralTheta = 0;
        double speedY = MaxYSpeed, speedX = MaxXSpeed, speedTheta = MaxThetaSpeed;
        double yPower,xPower,thetaPower;
        while (true && opModeIsActive()) {
            double currentY = myOtos.getPosition().y;
            double currentX = myOtos.getPosition().x;
            double currentTheta = myOtos.getPosition().h;
            double errorY = targetYPos - currentY;
            double errorX = targetXPos - currentX;
            double errorTheta = targetThetaPos - currentTheta;


            if ((MaxYSpeed == 0 || Math.abs(errorY) <= 0.75) &&
                    (MaxXSpeed == 0 || Math.abs(errorX) <= 0.75) &&
                    (MaxThetaSpeed == 0 || Math.abs(errorTheta) <= 10)) {
                break;
            }


            integralY = integralY + errorY;
            integralX = integralX + errorX;
            integralTheta = integralTheta + errorTheta;

            double derivativeY = errorY - previousErrorY;
            double derivativeX = errorX - previousErrorX;
            double derivativeTheta = errorTheta - previousErrorTheta;

            if (errorY < 0) {
                yPower = Math.max((Kp * errorY) + (Ki * integralY) + (Kd * derivativeY), -Math.abs(speedY));
            }
            else{
                yPower = Math.min((Kp * errorY) + (Ki * integralY) + (Kd * derivativeY), Math.abs(speedY));
            }

            if (errorX < 0) {
                xPower = Math.max((Kp * errorX) + (Ki * integralX) + (Kd * derivativeX), -Math.abs(speedX));
            }
            else{
                xPower = Math.min((Kp * errorX) + (Ki * integralX) + (Kd * derivativeX), Math.abs(speedX));
            }
            if (errorTheta < 0) {
                thetaPower = Math.max((KpTheta * errorTheta) + (KiTheta * integralTheta) + (KdTheta * derivativeTheta), -Math.abs(speedTheta));
            }
            else{
                thetaPower = Math.min((KpTheta * errorTheta) + (KiTheta * integralTheta) + (KdTheta * derivativeTheta), Math.abs(speedTheta));
            }

            previousErrorY = errorY;
            previousErrorX = errorX;
            previousErrorTheta = errorTheta;

            telemetry.addData("YPos", myOtos.getPosition().y);
            telemetry.addData("XPos", myOtos.getPosition().x);
            telemetry.addData("ThetaPos", myOtos.getPosition().h);
            telemetry.update();
//            Send calculated power to drivetrain

            fieldCentricDrivetrainControl((float) yPower, (float) xPower, (float) -thetaPower);
        }

    }
    private void stopAllMotors(){
        drivetrainControl(0,0,0);
    }
    private void viperControl(double viperSpeed){
        leftViperSlide.setPower(viperSpeed);
        rightViperSlide.setPower(-viperSpeed);
    }
    private void viperRunUp(double viperSpeed){

    }
    private void viperRunDown(double viperSpeed){

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
        frontRightStrafe = Range.clip(drive - strafe - turn, -1, 1);
        frontLeftStrafe = Range.clip(drive - strafe + turn, -1, 1);
        rearRightStrafe = Range.clip(drive + strafe - turn, -1, 1);
        rearLeftStrafe = Range.clip( drive + strafe + turn, -1, 1);


        frontLeftDrive.setPower(frontLeftStrafe);
        frontRightDrive.setPower(frontRightStrafe);
        rearLeftDrive.setPower(rearLeftStrafe);
        rearRightDrive.setPower(rearRightStrafe);
    }

    private void fieldCentricDrivetrainControl(float drive, float strafe, float turn) {
        double R = Math.sqrt(Math.pow(strafe,2) + Math.pow(drive,2));
        double theta = Math.atan2(drive,strafe);
        double correctedTheta = theta - Math.toRadians(myOtos.getPosition().h);
        double Y = R * Math.sin(correctedTheta);
        double X = R * Math.cos(correctedTheta);
        frontRightStrafe = (float) Range.clip(Y - X - turn, -1, 1);
        frontLeftStrafe = (float) Range.clip(Y - X + turn, -1, 1);
        rearRightStrafe = (float) Range.clip(Y + X - turn, -1, 1);
        rearLeftStrafe = (float) Range.clip( Y + X + turn, -1, 1);


        frontLeftDrive.setPower(frontLeftStrafe);
        frontRightDrive.setPower(frontRightStrafe);
        rearLeftDrive.setPower(rearLeftStrafe);
        rearRightDrive.setPower(rearRightStrafe);
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
            SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 1.89, 0);
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
            myOtos.setLinearScalar(0.826);
            myOtos.setAngularScalar(1);

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
            //TODO: Measure the distance from starting pos to the scoring position

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