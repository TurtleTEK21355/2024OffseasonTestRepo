/* Copyright (c) 2022 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WeirdAuto", group="Linear OpMode")
public class WeirdAuto extends LinearOpMode {
    private SparkFunOTOS myOtos;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;

    @Override
    public void runOpMode() {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        configureOtos();
        myOtos.resetTracking();
        waitForStart();

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        telemetry.addLine("this works");

        while(valueNotRoughlyEqual(pos.x, 5, 0.5) && opModeIsActive()){
            rearRightDrive.setPower(1);
            rearLeftDrive.setPower(1);
            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
            pos = myOtos.getPosition();
            telemetry.addLine("you're in the while loop");
            telemetry.addData("x:",pos.x);
            telemetry.update();
        }

        while (opModeIsActive()){
            pos = myOtos.getPosition();
            telemetry.addData("x:",pos.x);
            telemetry.addData("y:",pos.y);
            telemetry.addData("h:",pos.h);
            telemetry.addData("opModeIsActive",opModeIsActive());
            telemetry.update();
            break;

        }

        telemetry.addLine("this also works");
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        telemetry.addLine("end of the line");
        telemetry.update();
        sleep(10000);
        //moveRobot(10,0,0,0.1, 0.5, 1.0);
    }

    public void moveRobot(double x,double y,double h,double speed,double posTolerance,double headingTolerance){
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        telemetry.addData("x:",pos.x);
        telemetry.addData("y:",pos.y);
        telemetry.addData("h:",pos.h);
        telemetry.addData("opModeIsActive",opModeIsActive());
        telemetry.update();

        while(valueNotRoughlyEqual(pos.x, x, posTolerance) && valueNotRoughlyEqual(pos.y, y, posTolerance) && valueNotRoughlyEqual(pos.h, h, headingTolerance)) {
            pos = myOtos.getPosition();

            double ySide = bangBangController(y, pos.y, speed);
            double xSide = bangBangController(x, pos.x, speed);
            double hyp = Math.sqrt(Math.pow(xSide, 2) + Math.pow(ySide, 2));
            double theta = Math.atan2(ySide, xSide);
            double correctedTheta = theta - pos.h;

            double drive;

            if (valueNotRoughlyEqual(pos.x, x, posTolerance)) {
                drive = hyp * Math.sin(correctedTheta);
            }
            else {
                drive = 0.0;
            }

            double strafe;

            if (valueNotRoughlyEqual(pos.y, y, posTolerance)) {
                strafe = hyp * Math.cos(correctedTheta);
            }
            else {
                strafe = 0.0;
            }

            double turn;

            if (valueNotRoughlyEqual(pos.h, h, headingTolerance)) {
                turn = bangBangController(h, pos.h, speed);
            }
            else {
                turn = 0.0;
            }

            double frontLeftPower = Range.clip(drive + strafe + turn, -1, 1);
            double frontRightPower = Range.clip(drive - strafe - turn, -1, 1);
            double rearLeftPower = Range.clip(drive - strafe + turn, -1, 1);
            double rearRightPower = Range.clip(drive + strafe - turn, -1, 1);

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);

            telemetry.addData("x:",pos.x);
            telemetry.addData("y:",pos.y);
            telemetry.addData("h:",pos.h);
            telemetry.addData("drive",drive);
            telemetry.addData("turn",turn);
            telemetry.addData("strafe",strafe);
            telemetry.addData("frontLeftPower",frontLeftPower);
            telemetry.addData("frontRightPower",frontRightPower);
            telemetry.addData("rearLeftPower",rearLeftPower);
            telemetry.addData("rearRightPower",rearRightPower);
            telemetry.update();
        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
    }


    public boolean valueNotRoughlyEqual(double value, double goal, double tolerance) {
        if ((value > goal + tolerance) && (value < goal - tolerance)){
            return true;
        }
        else {
            return false;
        }
    }

    public double bangBangController(double goalPos, double currentPos,double speed) {
        double output = 0;

        if (currentPos < goalPos) {
            output = speed;
        } else if (currentPos > goalPos) {
            output = -speed;
        }

        return output;
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(0.9961);
        myOtos.setAngularScalar(0.9899);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

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