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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomusBlue", group="autonomous")  // @Autonomous(...) is the other common choice
public class AutonomousBlue extends LinearVisionOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Drive drive;
    DcMotor motorGun1;
    DcMotor motorGun2;
    //VisionRectangle recrec;
    int sleepTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();


        //sets up gun and drive motors
        drive = new Drive(hardwareMap, "gyro", telemetry);
        drive.resetEncoders();
        motorGun1 = hardwareMap.dcMotor.get("gun 1");
        motorGun2 = hardwareMap.dcMotor.get("gun 2");
        motorGun1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGun1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorGun2.setDirection(DcMotorSimple.Direction.FORWARD);

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
        //this.recrec = new VisionRectangle();

        /*waits for start after 10 seconds and allows you to set initial delay
        while (runtime.seconds() < 10 && opModeIsActive()) {
            if (gamepad1.a) {
                sleepTime += 10;
            } else if (gamepad1.b) {
                sleepTime -= 10;
            }
            telemetry.addData("waitTime: ", sleepTime);
        }*/
        // Setup vision rectangle with specified image pattern
        //      this.recrec.setup("legos");
        waitForStart();
        //sleep(sleepTime);

        //drives diagonally towards beacon
        drive.setValues(1, 1, 0);
        while (drive.driveToPosition(8600, .5) && opModeIsActive()) {}

        //moves in to get camera in better location
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(800, .4) && opModeIsActive()) {}

        drive.setValues(0, 0, 0);
/*
        // Grab the corners, and their average X value, of the image
        Mat output = new Mat();
        while (output.empty() && opModeIsActive()) {
            if (hasNewFrame()) {
                output = this.recrec.processFrame(getFrameRgba(), getFrameGray());
                discardFrame();
            }
        }

        Point p1 = new Point(output.get(0, 0));
        Point p2 = new Point(output.get(1, 0));
        Point p3 = new Point(output.get(2, 0));
        Point p4 = new Point(output.get(3, 0));
        double avgX = (p1.x + p2.x + p3.x + p4.x) / 4.0;
        double move = (avgX-450);
        if (move > 0) {
            drive.setValues(0, -1, 0);
        } else{
            drive.setValues(0, 1, 0);
        }

        while (drive.driveToPosition(move,  .5) && opModeIsActive()) {}
*/
        //senses beacon color and moves to that side
        sleep(400);
        boolean done = false;
        String s;
        while (!done && opModeIsActive()) {
            s = beacon.getAnalysis().getColorString();
            telemetry.addData("Color", s);
            if (s.equals("red, blue")) {
                drive.setValues(0, -1, 0);
                done = true;
            } else if (s.equals("blue, red")) {
                drive.setValues(0, 1, 0);
                done = true;
            } else {
                telemetry.addData("Beacon Analysis ", "Failed");
            }
        }

        while (drive.driveToPosition(150, .3) && opModeIsActive()) {}

        //presses button then moves away
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(1700, .4) && opModeIsActive()) {}

        drive.setValues(-1, 0, 0);
        while (drive.driveToPosition(1000, .5) && opModeIsActive()) {}

        //turns toward center goal
        drive.setValues(0, 0, -1);
        while (drive.driveToPosition(3900, .4) && opModeIsActive()) {}

        //fires gun
        motorGun1.setPower(.5);
        motorGun2.setPower(.5);
        sleep(5000);
        // wat
        motorGun2.setPower(0);
        motorGun1.setPower(0);

        drive.reset(0);

        //moves to hit cap ball
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(4500, 1) && opModeIsActive()) {}

        //rotates to pull cap ball of base plate
        drive.setValues(0, 0, -1);

        while (drive.driveToPosition(5000, 1) && opModeIsActive()) {}
    }
}
