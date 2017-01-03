/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class TeleOpOmni extends OpMode {
    /* Declare OpMode members. */
    //private ElapsedTime runtime = new ElapsedTime();

    Drive drive;

    DcMotor motorGun1;

    DcMotor motorIntake;

    DcMotor motorArm;
    DcMotor motorArm2;

    Servo servoButton;

    boolean driveN;

    private static final float SLOWEST_SPEED_FACTOR = 6;
    private static final float MIDDLE_SPEED_FACTOR = 4;
    private static final float FASTEST_SPEED_FACTOR = 1;

    //CURRENTLY ARBITRARY, NEEDS REFINEMENT
    public static final double BUTTON_MIDDLE = .35;
    public static final double BUTTON_LEFT = 0.1;
    public static final double BUTTON_RIGHT = .6;

    double speedFactor = FASTEST_SPEED_FACTOR;

    private boolean runCatapult = false;

    /***** BUTTONS *****
    GAMEPAD1:   Left stick x, y - movement
                Right stick x - rotation
                A - reset gyro
                Dpad up, down, left, right - adjust robot
                Right, left trigger - intake
                X, Y, B - speed of robot
                Start - reset

                Free controls: Right/left bumpers

     GAMEPAD2:  Right, left trigger - intake
                Left stick y - gun
                A - cock catapult
                X, B - button pusher (left, right)

                Free controls: Right stick x/y, Right/left bumper, Y, B, X, Dpad, Start, Right/left bumpers
     ******************/

    @Override
    public void loop() {
        //Gets joystick values and saves them to global variables.
        drive.xComp = gamepad1.left_stick_x;
        drive.yComp = -1 * gamepad1.left_stick_y;
        drive.rot = gamepad1.right_stick_x;

        if (drive.xComp < .05 && drive.xComp > -.05) {
            drive.xComp = 0;
        }
        if (drive.yComp < .05 && drive.yComp > -.05) {
            drive.yComp = 0;
        }

        if (drive.yComp == 0 && drive.xComp == 0) {
            driveN = true;
        }else {
            driveN = false;
        }

        if (drive.rot < .05 && drive.rot > -.05) {
            drive.rot = 0;
        }

        if (gamepad1.dpad_up && !gamepad1.a) {
            drive.yComp += 1;
        } else if (gamepad1.dpad_down && !gamepad1.a) {
            drive.yComp -= 1;
        } else if (gamepad1.dpad_left && !gamepad1.a) {
            drive.xComp -= 1;
        } else if (gamepad1.dpad_right && !gamepad1.a) {
            drive.xComp += 1;
        }

        if (gamepad1.dpad_up && gamepad1.a) {
            drive.reset(0);
        } else if (gamepad1.dpad_down && gamepad1.a) {
            drive.reset(180);
        } else if (gamepad1.dpad_left && gamepad1.a) {
            drive.reset(90);
        } else if (gamepad1.dpad_right && gamepad1.a) {
            drive.reset(270);
        }

        drive.useGyro();

        //This \/ is the controller dead zone
        if (drive.rot < .05 && drive.rot > -.05) {
            drive.rot = 0;
        }

        //This \/ sets a maximum speed (of 1) if you're rotating and driving simultaneously
        double speed = Math.sqrt(drive.xComp * drive.xComp + drive.yComp * drive.yComp) + Math.abs(drive.rot);
        if (speed > 1) {
            speed = 1;
        }

        double right1 = gamepad1.right_trigger / 1.8;
        double left1 = gamepad1.left_trigger / 1.8;
        double right2 = gamepad2.right_trigger / 1.8;
        double left2 = gamepad2.left_trigger / 1.8;

        //Dead zone
        if (right1 < .05) { right1 = 0; }
        if (left1 < .05) { left1 = 0; }
        if (right2 < .05) { right2 = 0; }
        if (left2 < .05) { left2 = 0; }

        //If you push the right trigger, the intake runs forwards (defaults to gamepad1) (defaults to forwards)
        if (right1 > 0) {
            motorIntake.setPower(right1);
        } else if (right2 > 0) {
            motorIntake.setPower(right2);
        }
        //If you push the left trigger on either, the intake runs backwards (defaults to gamepad1)
        else if (left1 > 0) {
            motorIntake.setPower(-left1);
        } else if (left2 > 0){
            motorIntake.setPower(-left2);
        } else {
            motorIntake.setPower(0);
        }

        //Catapult
        if (gamepad2.a) {
            runCatapult = true;
            motorArm.setPower(.75);
            motorArm2.setPower(.75);
        }
        if (runCatapult && motorArm.getCurrentPosition() >= 500) {
            runCatapult = false;
            motorArm.setPower(0);
            motorArm2.setPower(0);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Button pusher
        if (gamepad2.x) { //left button
            servoButton.setPosition(BUTTON_LEFT);
        }
        else if (gamepad2.b) { //right button
            servoButton.setPosition(BUTTON_RIGHT);
        }
        else { //middle
            servoButton.setPosition(BUTTON_MIDDLE);
        }

        //Gun at full power
        double power =  -gamepad2.left_stick_y;
        power = Range.clip(power, 0, 1);

        //Since the gears are interlocking, one motor needs to run backwards **Motor is set to reverse, so it still takes positive values**
        motorGun1.setPower(power);

        //If you push X, you get the slowest speed
        //If you push Y, you get the middle speed
        //If you push B, you get the fastest speed
        if (gamepad1.x) {
            speedFactor = SLOWEST_SPEED_FACTOR;
        } if (gamepad1.y) {
            speedFactor = MIDDLE_SPEED_FACTOR;
        } if (gamepad1.b) {
            speedFactor = FASTEST_SPEED_FACTOR;
        }
        speed = speed/speedFactor;

        //Sets field oriented drive or not. driveN is non-field oriented drive
        if (driveN) {
            drive.drive(speed, false, false);
        }else{
            drive.drive(speed, false, true);
        }
    }

    @Override
    public void init() {
        drive = new Drive(hardwareMap, "gyro", telemetry);
//brendan is a meme
        drive.resetEncoders();
        drive.runWithoutEncoders();

        motorGun1 = hardwareMap.dcMotor.get("gun 1");

        motorIntake = hardwareMap.dcMotor.get("intake");

        motorArm = hardwareMap.dcMotor.get("catapult");
        motorArm2 = hardwareMap.dcMotor.get("catapult 2");

        servoButton = hardwareMap.servo.get("button");

        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm2.setDirection(DcMotorSimple.Direction.REVERSE);


        servoButton.setPosition(BUTTON_MIDDLE);
    }

}
