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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Ur mom", group = "Iterative Opmode")
public class TeleOpOmni extends OpMode {
    /* Declare OpMode members. */
    //private ElapsedTime runtime = new ElapsedTime();

    Drive drive;

    DcMotor motorGun1;
    DcMotor motorGun2;

    DcMotor ballIntake;

    double intake = 0;

    @Override
    public void loop() {
        //Gets joystick values and saves them to global variables.
        drive.xComp = gamepad1.left_stick_x;
        drive.yComp = -1 * gamepad1.left_stick_y;
        drive.rot = gamepad1.right_stick_x;

        if (gamepad1.a) {
            drive.reset();
        }

        drive.useGyro();

        //This \/ is the controller dead zone
        if (drive.rot < .05 && drive.rot > -.05) {
            drive.rot = 0;
        }
        if (drive.xComp < .05 && drive.xComp > -.05) {
            drive.xComp = 0;
        }
        if (drive.yComp < .05 && drive.yComp > -.05) {
            drive.yComp = 0;
        }
        //This \/ sets a maximum speed (of 1) if you're rotating and driving simultaneously
        double speed = Math.sqrt(drive.xComp * drive.xComp + drive.yComp * drive.yComp) + Math.abs(drive.rot);
        if (speed > 1) {
            speed = 1;
        }

        //If you push both bumpers, you get the really slow mode
        //If you push only one bumper, you get the slow mode
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            speed = speed / 12;
        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
            speed = speed / 4;
        }

        //Gun
        double power =  -gamepad2.left_stick_y / 2;
        power = Range.clip(power, 0, 1);

        //Since the gears are interlocking, one motor needs to run backwards
        motorGun1.setPower(power);
        motorGun2.setPower(power);

        drive.drive(speed);

        if (gamepad2.x) {
            intake = -1;
        } if (gamepad2.y) {
            intake = 0;
        } if (gamepad2.b) {
            intake = 1;
        }

        ballIntake.setPower(intake);
    }

    @Override
    public void init() {
        drive = new Drive(hardwareMap, "gyro", telemetry);

        motorGun1 = hardwareMap.dcMotor.get("gun 1");
        motorGun2 = hardwareMap.dcMotor.get("gun 2");

        ballIntake = hardwareMap.dcMotor.get("intake");
        ballIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorGun2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}
