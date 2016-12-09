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

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class TeleOpOmni extends OpMode {
    /* Declare OpMode members. */
    //private ElapsedTime runtime = new ElapsedTime();

    Drive drive;

    DcMotor motorGun1;
    DcMotor motorGun2;

    DcMotor motorIntake;

    DcMotor motorArm;

    private final static float SLOWEST_SPEED_FACTOR = 6;
    private final static float MIDDLE_SPEED_FACTOR = 4;
    private final static float FASTEST_SPEED_FACTOR = 1;

    double speedFactor = FASTEST_SPEED_FACTOR;

    @Override
    public void loop() {
        //Gets joystick values and saves them to global variables.
        drive.xComp = gamepad1.left_stick_x;
        drive.yComp = -1 * gamepad1.left_stick_y;
        drive.rot = gamepad1.right_stick_x;

        if (gamepad1.a) {
            drive.reset();
        }

        if (drive.rot < .05 && drive.rot > -.05) {
            drive.rot = 0;
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

        boolean rightB1 = gamepad1.right_bumper;
        boolean rightB2 = gamepad2.right_bumper;
        boolean leftB1 = gamepad1.left_bumper;
        boolean leftB2 = gamepad2.left_bumper;

        //If a right and a left on any combination of controllers are being pushed, this is true.
        boolean conflict = (rightB1 && leftB1) || (rightB2 && leftB2) ||
                (rightB1 && leftB2) || (leftB1 && rightB2);

        //If a right button is being pushed and there are no conflicts, go forwards
        if ((rightB1 || rightB2) && !conflict) {
            motorArm.setPower(1);
        } //If a left button is being pushed and there are no conflicts, go backwards
        else if ((leftB1 || leftB2) && !conflict) {
            motorArm.setPower(-1);
        } //If there are conflicts or no buttons are being pushed, stop the arm
        else {
            motorArm.setPower(0);
        }

        //Gun
        double power =  -gamepad2.left_stick_y / 2;
        power = Range.clip(power, 0, .5);

        //Since the gears are interlocking, one motor needs to run backwards
        motorGun1.setPower(power);
        motorGun2.setPower(power);

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

        drive.drive(speed, false);
    }

    @Override
    public void init() {
        drive = new Drive(hardwareMap, "gyro", telemetry);

        motorGun1 = hardwareMap.dcMotor.get("gun 1");
        motorGun2 = hardwareMap.dcMotor.get("gun 2");

        motorIntake = hardwareMap.dcMotor.get("intake");

        motorArm = hardwareMap.dcMotor.get("arm");

        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorGun2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
