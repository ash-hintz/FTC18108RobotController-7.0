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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision1;
import org.firstinspires.ftc.teamcode.Vision.dataFromOpenCV;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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

@Autonomous(name="BlueRight")
// @Disabled
public class BlueRight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motorA = null;
    private DcMotor motorC = null;
    private Servo servoA = null;
    private CRServo servoB = null;
    private CRServo servoC = null;
    private CRServo servoD = null;
    private BNO055IMU imu;
    private DistanceSensor ds1;
    Orientation lastAngles = new Orientation();
    double globalAngle, startAngle, endAngle, currentAngle;
    double armPower;
    int shippingLevel = 0;
    int firstLevel = 550;
    int secondLevel = 1050;
    int thirdLevel = 1600;

    OpenCvCamera webcam;

    DistanceSensor distancion;

    EasyOpenCVVision1 pipeline;

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
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

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
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
        resetAngle();

        currentAngle = getAngle();
        startAngle = currentAngle;

        if (angle <= 0) {
            // Start Right turn
            motor0.setPower(power);
            motor1.setPower(-1 * power);
            motor2.setPower(power);
            motor3.setPower(-1 * power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle <= 0.4 * angle) {
                    motor0.setPower(0.4 * power);
                    motor1.setPower(-0.4 * power);
                    motor2.setPower(0.4 * power);
                    motor3.setPower(-0.4 * power);
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
                telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();
            }
        } else {
            // Start Left turn
            motor0.setPower(-1 * power);
            motor1.setPower(power);
            motor2.setPower(-1 * power);
            motor3.setPower(power);

            while (true) {
                currentAngle = getAngle();

                if (currentAngle >= 0.4 * angle) {
                    motor0.setPower(-0.4 * power);
                    motor1.setPower(0.4 * power);
                    motor2.setPower(-0.4 * power);
                    motor3.setPower(0.4 * power);
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
                telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
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

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetAngle();

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
                telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();

                // Stop driving when Motor Encoder Avg. >= motorDistance
                if ((motor0.getCurrentPosition() + motor1.getCurrentPosition() + motor2.getCurrentPosition() + motor3.getCurrentPosition()) / 4 >= motorDistance) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
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
                telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                        motor0.getCurrentPosition(),
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition());
                telemetry.update();

                // Stop driving when Motor Encoder Avg. <= motorDistance
                if ((motor0.getCurrentPosition() + motor1.getCurrentPosition() + motor2.getCurrentPosition() + motor3.getCurrentPosition()) / 4 <= motorDistance) {
                    motor0.setPower(0.0);
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    break;
                }
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //  Setup IMU configurations
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        ds1 = hardwareMap.get(DistanceSensor.class, "ds1");
        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        int startPos = 0;
        int endPos = 0;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        EasyOpenCVVision1 pipeline = new EasyOpenCVVision1();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        // servoA = hardwareMap.get(Servo.class, "servoA");
        servoA = hardwareMap.get(Servo.class, "servoA");
        servoB = hardwareMap.get(CRServo.class, "servoB");
        servoC = hardwareMap.get(CRServo.class, "servoC");
        servoD = hardwareMap.get(CRServo.class, "servoD");



        // Setup DC Motor configurations
        // Most robots need the motor on one side to be reversed to drive forward
        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motorA.setDirection(DcMotor.Direction.FORWARD);
        motorC.setDirection(DcMotor.Direction.FORWARD);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // make sure the IMU gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        {
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.addData("avg1", dataFromOpenCV.AVG1);
            telemetry.addData("avg2", dataFromOpenCV.AVG2);
            telemetry.update();
            //TODO
            //sleep(10000);
            int ShElementPosition = 10;
            if ((pipeline.position == EasyOpenCVVision1.ShipPosition.LEFT)) {
                ShElementPosition = 1;
            }
            if ((pipeline.position == EasyOpenCVVision1.ShipPosition.CENTER)) {
                ShElementPosition = 2;
            }
            if ((pipeline.position == EasyOpenCVVision1.ShipPosition.NONE)) {
                ShElementPosition = 3;
            }
            //Voltage regulation depending on the battery charge level
            telemetry.addData("ShElementPosition", ShElementPosition);
            telemetry.update();
            boolean check = true;

            //lifttime.reset();

            // START AUTONOMOUS PROGRAM

            servoA.setPosition(0.10);
            sleep(1250);
            driveStraightGyro(200, 0.3);
            sleep(1000);

            if (ShElementPosition == 1 && check) {
                turnTankGyro(49, 0.3);
                sleep(500);
                driveStraightGyro(800, 0.6);
                sleep(500);
                turnTankGyro(-10, 0.15);
                sleep(400);
                while (true) {
                    motorA.setTargetPosition(firstLevel);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() >= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
                driveStraightGyro(300, 0.2);
                sleep(300);
                servoA.setPosition(0.25);
                sleep(400);
                driveStraightGyro(-380, 0.5);
                turnTankGyro(45, 0.5);
                driveStraightGyro(-1280, 0.7);
                sleep(400);
                turnTankGyro(-75, 0.4);
                driveStraightGyro(-550, 0.3);
                sleep(400);
                motorC.setPower(-0.07);
                while (true) {
                    if (motorC.getCurrentPosition() <= -450) {
                        motorC.setPower(0.0);
                        break;
                    }
                }
                sleep(400);
                driveStraightGyro(125, 0.3);
                sleep(400);
                turnTankGyro(-10, 0.5);
                sleep(400);
                driveStraightGyro(600, 0.6);
                sleep(400);
                servoA.setPosition(0.10);
                sleep(750);
                while (true) {
                    motorA.setTargetPosition(-50);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() <= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
            }

            if (ShElementPosition == 2 && check) {
                turnTankGyro(39, 0.3);
                sleep(500);
                driveStraightGyro(725, 0.6);
                sleep(500);
                while (true) {
                    motorA.setTargetPosition(secondLevel);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() >= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
                driveStraightGyro(265, 0.2);
                sleep(300);
                servoA.setPosition(0.25);
                sleep(400);
                driveStraightGyro(-380, 0.5);
                turnTankGyro(45, 0.5);
                driveStraightGyro(-1230, 0.7);
                sleep(400);
                turnTankGyro(-75, 0.4);
                driveStraightGyro(-550, 0.3);
                sleep(400);
                motorC.setPower(-0.07);
                while (true) {
                    if (motorC.getCurrentPosition() <= -450) {
                        motorC.setPower(0.0);
                        break;
                    }
                }
                sleep(400);
                driveStraightGyro(125, 0.3);
                sleep(400);
                turnTankGyro(-10, 0.5);
                sleep(400);
                driveStraightGyro(600, 0.6);
                sleep(400);
                servoA.setPosition(0.10);
                sleep(750);
                while (true) {
                    motorA.setTargetPosition(-50);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() <= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
            }

            if (ShElementPosition == 3 & check) {
                turnTankGyro(39, 0.3);
                sleep(500);
                driveStraightGyro(765, 0.6);
                sleep(400);
                while (true) {
                    motorA.setTargetPosition(thirdLevel);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() >= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
                driveStraightGyro(310, 0.2);
                sleep(300);
                servoA.setPosition(0.25);
                sleep(400);
                driveStraightGyro(-380, 0.5);
                turnTankGyro(45, 0.5);
                driveStraightGyro(-1280, 0.7);
                sleep(400);
                turnTankGyro(-78, 0.4);
                driveStraightGyro(-550, 0.3);
                sleep(400);
                motorC.setPower(-0.07);
                while (true) {
                    if (motorC.getCurrentPosition() <= -450) {
                        motorC.setPower(0.0);
                        break;
                    }
                }
                sleep(400);
                driveStraightGyro(125, 0.3);
                sleep(400);
                turnTankGyro(-7, 0.5);
                sleep(400);
                driveStraightGyro(600, 0.6);
                sleep(400);
                servoA.setPosition(0.10);
                sleep(750);
                while (true) {
                    motorA.setTargetPosition(-50);
                    motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorA.setPower(0.4);
                    if (motorA.getCurrentPosition() <= motorA.getTargetPosition()) {
                        motorA.setPower(0.0);
                        break;
                    }
                }
            }

            // END AUTONOMOUS PROGRAM
        }

        while (opModeIsActive()) {
            telemetry.addData("Duck Position: ", pipeline.position);
            telemetry.addData("avg1", dataFromOpenCV.AVG1);
            telemetry.addData("avg2", dataFromOpenCV.AVG2);
            telemetry.addData("avg3", dataFromOpenCV.AVG3);
            telemetry.update();
        }



        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        // Send telemetry message to indicate successful Encoder reset
        // telemetry.setAutoClear(false);
        telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
                motor0.getCurrentPosition(),
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition());
        telemetry.update();
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(10, 10),
                    new Point(70, 200),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.z
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARGTPc3/////AAABmQPsjNjUvkhEpOekRVJS6hFuyqQd8MDqXOad0+ftSCpchv3jSVN+LPwH9lupDnWlh91rhm3M2+DFMZ9QktxCXz/8BYKSHVvfqlqPnHwLLO+xHrcd9MxtmUSG6HvzphpqmAq+2plmXSIDCRJ1f8qgMDFXy3S3LVFzGGm95EwOp/wr1FtolYp+MqrYwqn90b2O5ZzGvYzi14WdmqKIlo3QqE9SRc3wZe/8TKTGXdYmM7rkWwpPqmlhSCEdQU09jSco6FQGX+WxPbv88CTeYbhn6UrDBWehTx7Ns+Hjoers9yrs0MWkWEaEeUCSuP+6R0UbuXOjrgr0vEmpcDZh2z2ARVKlqXvOKzAr66TFHBWA6uH2";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
}


