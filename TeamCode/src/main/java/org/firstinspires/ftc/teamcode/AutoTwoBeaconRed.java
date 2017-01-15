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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoRed2B", group="autonomous")  // @Autonomous(...) is the other common choice
public class AutoTwoBeaconRed extends LinearVisionOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Drive drive;
    DcMotor motorGun1;

    Servo servoButton;
    Servo armRelease;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();

        //sets up gun and drive motors
        drive = new Drive(hardwareMap, "gyro", telemetry);
        drive.resetEncoders();
        motorGun1 = hardwareMap.dcMotor.get("gun 1");
        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun1.setDirection(DcMotorSimple.Direction.FORWARD);

        //initialize servo
        servoButton = hardwareMap.servo.get("button");
        armRelease = hardwareMap.servo.get("arm servo");
        servoButton.setPosition(TeleOpOmni.BUTTON_MIDDLE);
        armRelease.setPosition(.5);

        //initializes camera
        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        enableExtension(Extensions.BEACON);
        enableExtension(Extensions.ROTATION);
        enableExtension(Extensions.CAMERA_CONTROL);
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        waitForStart();

        //drives at diagonal to first beacon
        drive.setValues(.63, -1, 0);
        while (drive.driveToPosition(8300, .8) && opModeIsActive()) {}

        //senses beacon color and moves to that side
        drive.xComp = 0;
        boolean done = false;
        String s;
        double pos = TeleOpOmni.BUTTON_MIDDLE; //the position to set the button pusher to
        while (!done && opModeIsActive()) {
            s = beacon.getAnalysis().getColorString();
            //telemetry.addData("Color", s);
            if (s.equals("red, blue")) {
                pos = TeleOpOmni.BUTTON_LEFT;
                done = true;
            } else if (s.equals("blue, red")) {
                pos = TeleOpOmni.BUTTON_RIGHT;
                done = true;
            }
        }

        //moves forwards to press button
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(1700, .3) && opModeIsActive()) {}

        //adjusts the button pusher
        servoButton.setPosition(pos);
        sleep(1000);
        servoButton.setPosition(TeleOpOmni.BUTTON_MIDDLE);

        //moves forwards to press button
        drive.setValues(-1, 0, 0);
        while (drive.driveToPosition(900, .4) && opModeIsActive()) {}

        //moves towards next beacon
        drive.setValues(-.07, -1, 0);
        while (drive.driveToPosition(5300, .6) && opModeIsActive()) {}

        //senses beacon color and moves to that side
        drive.xComp = 0;
        done = false;
        pos = TeleOpOmni.BUTTON_MIDDLE; //the position to set the button pusher to
        while (!done && opModeIsActive()) {
            s = beacon.getAnalysis().getColorString();
            //telemetry.addData("Color", s);
            if (s.equals("red, blue")) {
                pos = TeleOpOmni.BUTTON_LEFT;
                done = true;
            } else if (s.equals("blue, red")) {
                pos = TeleOpOmni.BUTTON_RIGHT;
                done = true;
            }
        }

        //moves forwards to press button
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(1200, .3) && opModeIsActive()) {}

        //adjusts the button pusher
        servoButton.setPosition(pos);
        sleep(1000);
        servoButton.setPosition(TeleOpOmni.BUTTON_MIDDLE);

        drive.reset(0);

        //rotates while moving toward center goal
        drive.setValues(-1, 0, 0);
        while (drive.driveToPosition(1500, .6) && opModeIsActive()) {}

        drive.setValues(0, .7, -.78);
        while (drive.driveToPosition(6500, .6) && opModeIsActive()) {}

        //fires gun
        motorGun1.setPower(1);
        sleep(3000);
        motorGun1.setPower(0);

        drive.reset(0);

        //moves and hits cap ball
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(3000, .6) && opModeIsActive()) {}

        //rotates to push cap ball off bse plate
        drive.setValues(0, 0, -1);
        while (drive.driveToPosition(1500, 1) && opModeIsActive()) {}
    }
}
