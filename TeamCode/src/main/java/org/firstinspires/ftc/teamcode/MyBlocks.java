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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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

@TeleOp(name="MyBlocks", group="Linear Opmode")
// @Disabled
public class MyBlocks extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motorA = null;
    private DcMotor motorB = null;
    private Servo servo;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorB = hardwareMap.get(DcMotor.class, "motorB");
        servo = hardwareMap.get(Servo.class, "wobbleGrip");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);

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

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // make sure the IMU gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Encoders:", "M0: %3d  M1:%3d  M2:%3d  M3:%3d",
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup variables used during driving loop
        // Drive wheel power to set motor speed and display telemetry
        boolean liftUp = false;
        boolean liftDown = false;
        boolean intakeOn = false;
        double liftPower;
        double intakePower;
        boolean toggleDriving; // True = Car Mode, False = Tank Mode
        double leftWheelPower;
        double rightWheelPower;
        boolean motorBState = false;
        boolean buttonXState = false;
        boolean buttonYState = false;
        double xAxis;
        double yAxis;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            xAxis = gamepad1.right_stick_x;
            yAxis = gamepad1.right_stick_y;
            leftWheelPower = 0.3 * Range.clip(xAxis, -1.0, 1.0);
            rightWheelPower = 0.3 * Range.clip(yAxis, -1.0, 1.0);


            // Start Ring Intake Motor

            if ((!buttonYState) && (gamepad1.y)) {
                if (!motorBState) {
                    motorBState = true;
                    buttonYState = true;
                    motorB.setPower(1.0);
                } else {
                    motorBState = false;
                    buttonYState = true;
                    motorB.setPower(0.0);
                }
            }
            if ((!gamepad1.y) && (buttonYState)) {
                buttonYState = false;
            }
        }
    }

    public void MoveTank(double lPower, double rPower) {
        double leftPower = lPower;
        double rightPower = rPower;

        motor0.setPower(leftPower * 0.4);
        motor1.setPower(rightPower * -0.4);
        motor2.setPower(leftPower * 0.4);
        motor3.setPower(rightPower * -0.4);
    }

    public double getAngle () {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", "(%.2f)", angles.firstAngle);
        telemetry.update();

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return (int) globalAngle;
    }

    // TurnTankGyro
    public void TurnTankGyro(int Angle, int Speed) {
        // By: Anirudh Jagannathan

        // IN THIS MYBLOCK, SPEED MUST ALWAYS BE POSITIVE!!!
        // Degrees can be negative or positive

        double power = -0.4;
        double motorDistance = 5000;
        double correction = 0;

        double my_speed = Speed;
        double my_angle = Angle;
        double start_angle = getAngle();
        double stop_angle = (start_angle + my_angle);
        double decel_angle = my_speed * 2.4;
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (my_angle >= 0) {
            // Turning right
            while (true) {
                double current_gyro_angle = getAngle();
                if ((current_gyro_angle) >= stop_angle) {
                    MoveTank(0, 0);
                    break;
                }
                if (current_gyro_angle >= (stop_angle - decel_angle)) {
                    double calc = (((start_angle + my_angle) - (current_gyro_angle)) / decel_angle);
                    double new_speed = my_speed * calc;
                    if (new_speed > 1) {
                        MoveTank(new_speed, (-1 * new_speed));
                    } else {
                        MoveTank(1, -1);
                    }
                }
                else {
                    MoveTank(my_speed, -1 * my_speed);
                }
            }
        }
        else {
            // Turning left
            while (true) {
                double current_gyro_angle = getAngle();
                if ((current_gyro_angle) <= stop_angle) {
                    MoveTank(0, 0);
                    break;
                }
                if (current_gyro_angle <= (stop_angle + decel_angle)) {
                    double calc = -1 * (((start_angle + my_angle) - (current_gyro_angle)) / decel_angle);
                    double new_speed = my_speed * calc;
                    if (new_speed > 1) {
                        MoveTank((-1 * new_speed), new_speed);
                    }
                    else {
                        MoveTank(-1, 1);
                    }
                }
                else {
                    MoveTank(-1 * my_speed, my_speed);
                }
            }
        }
    }
}