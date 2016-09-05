/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="TeleOp")
//@Disabled
public class TeleOp extends OpMode {

    //positions for the servo to be set to at the end of each loop
    double armPos;
    double climberPos;
    double leftClimberPos;
    double rightClimberPos;
    double guardPos;
    double lifterPos;
    //lift speed
    double liftPower;

    //initializes all motors
    DcMotor motorRight1;
    DcMotor motorLeft2;
    DcMotor motorRight3;
    DcMotor motorLeft4;
    DcMotor motorRight5;
    DcMotor motorLeft6;
    DcMotor motorLift1;
    DcMotor motorLift2;

    //initializes all servos
    Servo climber;
    Servo arm;
    Servo leftClimber;
    Servo rightClimber;
    Servo guard;
    Servo lifter;

    boolean playerJoy;
    boolean temp;

    @Override
    public void init() {
        playerJoy = true;
        temp = false;

        //sets all motors made in the config file set to sides and numbers
        motorRight1 = hardwareMap.dcMotor.get("motor_1");
        motorLeft2 = hardwareMap.dcMotor.get("motor_2");
        motorRight3 = hardwareMap.dcMotor.get("motor_3");
        motorLeft4 = hardwareMap.dcMotor.get("motor_4");
        motorRight5 = hardwareMap.dcMotor.get("motor_5");
        motorLeft6 = hardwareMap.dcMotor.get("motor_6");
        //sets lift motors to lift1(right), and lift2(left)
        motorLift1 = hardwareMap.dcMotor.get("motor_7");
        motorLift2 = hardwareMap.dcMotor.get("motor_8");

        //sets all motors directions - right motors forward, left backward
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight3.setDirection(DcMotor.Direction.FORWARD);
        motorLeft4.setDirection(DcMotor.Direction.REVERSE);
        motorRight5.setDirection(DcMotor.Direction.FORWARD);
        motorLeft6.setDirection(DcMotor.Direction.REVERSE);
        motorLift1.setDirection(DcMotor.Direction.FORWARD);
        motorLift2.setDirection(DcMotor.Direction.REVERSE);

        //runmode of lift is set to run without encoders so the motors do not fight
        motorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //sets servos specified in config file to their name
        arm = hardwareMap.servo.get("arm");
        climber = hardwareMap.servo.get("climbers");
        leftClimber = hardwareMap.servo.get("servor");
        rightClimber = hardwareMap.servo.get("servol");
        guard = hardwareMap.servo.get("guard");
        lifter = hardwareMap.servo.get("lifter");

        //sets the starting position of the servos
        armPos = .41;
        climberPos = .8;
        leftClimberPos = .5;
        rightClimberPos = .5;
        guardPos = .59;
        lifterPos = 0;

        //sets the starting lift speed
        liftPower = 0;
    }

    @Override
    public void loop() {

        /*receives values fro them driver's joystick,
        since we are using tank drive,
        the right joystick sets the speed of the right motors,
        and the left joystick controls the motors on the left*/
        float right = -gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_y;

        if (playerJoy) {

            //clips joystick values to be within range of the motor speed
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            //drive slowmodes, each bumper halving the robot's speed
            if (gamepad1.right_bumper) {
                right /= 2;
                left /= 2;
            }
            if (gamepad1.left_bumper) {
                right /= 2;
                left /= 2;
            }

            //sets speed of motors to the left and right variables
            move(right, left);

            //moves the climber dropper based on operator's bumpers
            if (gamepad1.left_bumper) {
                climberPos += .001;
            } else if (gamepad1.right_bumper) {
                climberPos -= .001;
            }

            //moves left zipliner dropper
            if (gamepad1.y) {
                leftClimberPos += .003;
            } else if (gamepad1.a) {
                leftClimberPos -= .003;
            }

            //moves right zipliner dropper
            if (gamepad1.x) {
                rightClimberPos += .003;
            } else if (gamepad1.b) {
                rightClimberPos -= .003;
            }

            //moves the debris gaurd to up or down
            //position based on the operator's dpad
            if (gamepad1.dpad_up) {
                guardPos = .59;
            } else if (gamepad1.dpad_down) {
                guardPos = 0;
            }
        } else {
            move(0,0);
        }

        if (gamepad2.start || temp) {
            temp = !temp;
            if (!gamepad2.start) {
                temp = !temp;
                playerJoy = !playerJoy;
            }
        }

        //sets arm elevation is operator's left joystick is up/down
        if (gamepad2.left_stick_y > .2) {
            armPos += .001;
        } else if (gamepad2.left_stick_y < -.2) {
            armPos -= .001;
        }

        //sets lift power to operator's  left joystick
        if (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1) {
            liftPower = gamepad2.right_stick_y;
        } else {
            liftPower = 0;
        }

        //clips range of servos within their range of motion
        armPos = Range.clip(armPos, 0, .41);
        climberPos = Range.clip(climberPos, 0, 1);
        leftClimberPos = Range.clip(leftClimberPos, 0, 1);
        rightClimberPos = Range.clip(rightClimberPos, 0, 1);
        lifterPos = Range.clip(lifterPos, 0, 1);

        //moves servos to the positions set earlier
        arm.setPosition(armPos);
        climber.setPosition(climberPos);
        leftClimber.setPosition(leftClimberPos);
        rightClimber.setPosition(rightClimberPos);
        guard.setPosition(guardPos);
        lifter.setPosition(lifterPos);

        //moves lift at the power set earlier
        motorLift1.setPower(liftPower);
        motorLift2.setPower(liftPower);

        //sends data back to the divers containing
        //servo positions, lift power, and drive motor powers
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "%.2f", armPos);
        telemetry.addData("climber", "%.2f", climberPos);
        telemetry.addData("left climber", "%.2f", leftClimberPos);
        telemetry.addData("right climber", "%.2f", rightClimberPos);
        telemetry.addData("front guard", "%.2f", guardPos);
        telemetry.addData("rear lifter", "%.2f", lifterPos);
        telemetry.addData("lift Power", "%.2f", liftPower);
        telemetry.addData("left tgt pwr", "%.2f", left);
        telemetry.addData("right tgt pwr", "%.2f", right);

    }

    //move function which sets all drive motors to run without encoders,
    //so that they do not fight each other. Then sets the speed of each
    //one to the imputed values of right speed and left speed respectively.
    public void move(double speedR, double speedL) {
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRight1.setPower(speedR);
        motorLeft2.setPower(speedL);
        motorRight3.setPower(speedR);
        motorLeft4.setPower(speedL);
        motorRight5.setPower(speedR);
        motorLeft6.setPower(speedL);
    }
}