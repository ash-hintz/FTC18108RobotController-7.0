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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueLeft_Autonomous")
// @Disabled
public class BlueLeft_Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0, motor1, motor2, motor3, motorA, motorC;
    private Servo servoA;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, startAngle, endAngle, currentAngle;

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                motor0.getCurrentPosition(),
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition());
        telemetry.update();

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .02;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void turnTankGyro(double angleToTurn, double anglePower) {
        double angle = angleToTurn;
        double power = anglePower;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                motor0.getCurrentPosition(),
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition());
        telemetry.update();

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentAngle = getAngle();
        startAngle = currentAngle;

        if (angle <= 0) {
            // Start Right turn
            motor0.setPower(power);
            motor1.setPower(-1*power);
            motor2.setPower(power);
            motor3.setPower(-1*power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle <= 0.4 * angle) {
                    motor0.setPower(0.4*power);
                    motor1.setPower(-0.4*power);
                    motor2.setPower(0.4*power);
                    motor3.setPower(-0.4*power);
                }

                // Stop turning when the turned angle = requested angle
                if (currentAngle <= angle) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
                telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();
            }
        }
        else {
            // Start Left turn
            motor0.setPower(-1*power);
            motor1.setPower(power);
            motor2.setPower(-1*power);
            motor3.setPower(power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle >= 0.4 * angle) {
                    motor0.setPower(-0.4*power);
                    motor1.setPower(0.4*power);
                    motor2.setPower(-0.4*power);
                    motor3.setPower(0.4*power);
                }

                // Stop turning when the turned angle = requested angle
                if (currentAngle >= angle) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
                telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void driveIntoWall() {
        double power = 0.3;

        // Start driving
        motor0.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);

        while (true) {
            // If bot left side has hit the wall (stopped) then shut down left side motors
            if ((motor2.getPower() < (power * 0.75))) {
                motor0.setPower(0.0);
                motor2.setPower(0.0);
            }
            // If bot right side has hit the wall (stopped) then shut down right side motors
            if ((motor3.getPower() < (power * 0.75))) {
                motor1.setPower(0.0);
                motor3.setPower(0.0);
            }
            // if both sides of the bot are stopped then exit the driving loop
            if ((motor0.getPower() == 0.0) && (motor1.getPower() == 0.0))
                break;
        }
    }

    public void driveStraightGyro(int degreesToDrive, double drivePower) {

        double power = drivePower;
        int motorDistance = degreesToDrive;
        double correction = 0.02;
        Orientation angle;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorDistance >= 0) {
            // Start driving forward
            motor0.setPower(power);
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);

            while (true) {
                // telemetry.addData("Motor0",  "Distance: %3d", motor0.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                motor0.setPower(power - correction);
                motor1.setPower(power + correction);
                motor2.setPower(power - correction);
                motor3.setPower(power + correction);

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((motor0.getCurrentPosition()+motor1.getCurrentPosition()+motor2.getCurrentPosition()+motor3.getCurrentPosition())/4 >= motorDistance) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.addData("IMU calib status", imu.getCalibrationStatus().toString());
                telemetry.update();
            }
        }
        if (motorDistance < 0) {
            // Start driving backward
            motor0.setPower(-power);
            motor1.setPower(-power);
            motor2.setPower(-power);
            motor3.setPower(-power);

            while (true) {
                // telemetry.addData("Motor0",  "Distance: %3d", motor0.getCurrentPosition());
                // telemetry.update();

                correction = checkDirection();
                motor0.setPower(-1 * (power + correction));
                motor1.setPower(-1 * (power - correction));
                motor2.setPower(-1 * (power + correction));
                motor3.setPower(-1 * (power - correction));

                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((motor0.getCurrentPosition()+motor1.getCurrentPosition()+motor2.getCurrentPosition()+motor3.getCurrentPosition())/4 <= motorDistance) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
                telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
                telemetry.update();
            }
        }
    }

    public void carouselTurn (double carouselDegrees, double carouselPower) {

        double cDegrees = carouselDegrees;
        double cPower = carouselPower;

        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Carousel moving left
        if (cDegrees <= 0) {

            while (true) {
                motorC.setPower(-cPower);
                telemetry.addData("Encoder:",  "MC: %3d",
                        motorC.getCurrentPosition());
                telemetry.update();

                if (motorC.getCurrentPosition() <= cDegrees) {
                    motorC.setPower(0.0);
                    break;
                }
            }
        }
        // Carousel moving right
        if (cDegrees > 0) {

            while (true) {
                motorC.setPower(cPower);

                if (motorC.getCurrentPosition() >= cDegrees) {
                    motorC.setPower(0.0);
                    break;
                }
            }
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        servoA = hardwareMap.get(Servo.class, "servoA");

        // Setup IMU configurations
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        // Setup DC Motor configurations
        // Most robots need the motor on one side to be reversed to drive forward
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motorA.setDirection(DcMotor.Direction.FORWARD);
        motorC.setDirection(DcMotor.Direction.FORWARD);


        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // make sure the IMU gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                motor0.getCurrentPosition(),
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition());
        telemetry.addData("IMU calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // START AUTONOMOUS PROGRAM

        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        servoA.setPosition(0.10);
        sleep(1250);
        motorA.setPower(0.7);
        while (true) {
            telemetry.addData("Encoder:", "MA: %3d", motorA.getCurrentPosition());
            telemetry.update();
            if (motorA.getCurrentPosition() >= 1050) {
                motorA.setPower(0.0);
                break;
            }
        }

        driveStraightGyro(250,0.5);
        sleep(250);
        turnTankGyro(-80, 0.5);
        imu.initialize(parameters);
        driveStraightGyro(850, 0.5);
        turnTankGyro(80, 0.5);

        motorA.setPower(0.7);
        while (true) {
            telemetry.addData("Encoder:", "MA: %3d", motorA.getCurrentPosition());
            telemetry.update();
            if (motorA.getCurrentPosition() >= 1100) {
                motorA.setPower(0.0);
                break;
            }
        }

        imu.initialize(parameters);
        driveStraightGyro(250, 0.3);
        sleep(250);
        servoA.setPosition(0.30);
        sleep(500);
        driveStraightGyro(-175, 0.5);
        sleep(250);
        turnTankGyro(80, 0.5);
        imu.initialize(parameters);
        driveStraightGyro(2650, 0.7);

        servoA.setPosition(0.10);
        motorA.setPower(-0.4);
        while (true) {
            telemetry.addData("Encoder:", "MA: %3d", motorA.getCurrentPosition());
            telemetry.update();
            if (motorA.getCurrentPosition() <= 0.0) {
                motorA.setPower(0.0);
                break;
            }
        }

        // END AUTONOMOUS PROGRAM


        // Send telemetry message to indicate successful Encoder reset
        // telemetry.setAutoClear(false);
        telemetry.addData("Encoders:",  "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                motor0.getCurrentPosition(),
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition());
        telemetry.update();
        sleep(10000);
    }
}

