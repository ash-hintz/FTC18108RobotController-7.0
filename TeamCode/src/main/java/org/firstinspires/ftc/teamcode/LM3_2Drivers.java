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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="LM3_2Drivers")
// @Disabled
public class LM3_2Drivers extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motorA = null;
    private DcMotor motorC = null;
    private Servo servoA;
    private CRServo servoB;
    // Define Servo class members
    static final double INCREMENT   = 0.03;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   30;     // period of each cycle

    static final double AMAX_POS     =  0.80;     // Maximum rotational position
    static final double AMIN_POS     =  0.10;     // Minimum rotational position
    double  Aposition = AMIN_POS;                 // Start position

    static final double BMAX_POS     =  0.50;     // Maximum rotational position
    static final double BMIN_POS     =  0.00;     // Minimum rotational position
    double  Bposition = BMIN_POS;                 // Start position

    // Variables to detect buttons pressed
    boolean gp2ButtonAPressed;
    boolean gp2ButtonBPressed;
    boolean gp2ButtonXPressed;
    boolean gp2ButtonYPressed;

    // Setup variables used during driving loop
    // Drive wheel power to set motor speed and display telemetry
    double leftPower;
    double rightPower;
    double armPower;
    double armHeightMin = -10;
    double armHeightMax = 1300;
    double carouselPower = 0.12;

    // Local variable to control Arm / Carousel / Class
    boolean armIsMoving = false;
    int armFloor = 0;
    int armShippingHubL1 = 350;
    int armShippingHubL2 = 700;
    int armShippingHubL3 = 1150;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorC = hardwareMap.get(DcMotor.class, "motorC");
        servoA = hardwareMap.get(Servo.class, "servoA");
        servoB = hardwareMap.get(CRServo.class, "servoB");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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

        // Set both servos to the starting position
        // servoA.setPosition(0.15);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;

            if (gamepad1.b) {
                leftPower = Range.clip(drive + turn, -0.2, 0.2);
                rightPower = Range.clip(drive - turn, -0.2, 0.2);
            }

            else {
                leftPower = Range.clip(drive + turn, -0.85, 0.85);
                rightPower = Range.clip(drive - turn, -0.85, 0.85);
            }

            // Send calculated power to wheels
            motor0.setPower(leftPower);
            motor1.setPower(rightPower);
            motor2.setPower(leftPower);
            motor3.setPower(rightPower);

            // Set arm the right
            double armJoyStick = -gamepad2.right_stick_y;
            armPower = Range.clip(armJoyStick, -0.5, 0.5);
            telemetry.addData("Arm", "Power (%.2f), Position (%3d)", armPower, motorA.getCurrentPosition());

            if ((motorA.getCurrentPosition() <= armHeightMin) && (armPower >= 0)) {
                motorA.setPower(0.0);
            }
            else if ((motorA.getCurrentPosition() >= armHeightMax) && (armPower <= 0)) {
                motorA.setPower(0.0);
            }
            else {
                motorA.setPower(-1 * armPower);
            }

            // Turn the Red Carousel On / Off

            if (gamepad2.b)
                motorC.setPower(carouselPower);
            if (gamepad2.x)
                motorC.setPower(0.0);

            // Turn the Blue Carousel On / Off
            if (gamepad2.y)
                motorC.setPower(-1*carouselPower);
            if (gamepad2.a)
                motorC.setPower(0.0);

            // Slew the servo, according to the rampUp (direction) variable.

            if (gamepad2.left_bumper) {

                // Keep stepping up until we hit the max value.
                Aposition += INCREMENT;
                if (Aposition >= AMAX_POS ) {
                    Aposition = AMAX_POS;
                }
            }

            if (gamepad2.right_bumper) {
                // Keep stepping down until we hit the min value.
                Aposition -= INCREMENT ;
                if (Aposition <= AMIN_POS) {
                    Aposition = AMIN_POS;
                }
            }

            if (gamepad2.dpad_up) {
                servoB.setPower(-1.0);
            }

            if (gamepad2.dpad_down) {
                servoB.setPower(1.0);
            }

            if (gamepad2.dpad_right) {
                servoB.setPower(0.0);
            }



            // Set the servo to the new position and pause;
            servoA.setPosition(Aposition);
            sleep(CYCLE_MS);
            idle();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status",  "Run Time: " + runtime.toString());
            telemetry.addData("Wheel Power", "Left (%.2f), Right (%.2f)", leftPower, rightPower);
            telemetry.addData("Arm", "Power (%.2f), Position (%3d)", armPower, motorA.getCurrentPosition());
            telemetry.addData("Carousel", "Power (%.2f)", carouselPower);
            telemetry.addData("Position Values", "A: (%.2f), B: (%.2f)", Aposition, Bposition);
            telemetry.update();
        }
    }
}
