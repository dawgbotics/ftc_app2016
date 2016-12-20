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
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomusTest", group="autonomous")  // @Autonomous(...) is the other common choice
public class AutonomousTest extends LinearVisionOpMode {
    ElapsedTime timer;
    VisionRectangle recrec;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        waitForVisionStart();

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
        this.recrec = new VisionRectangle();


        timer.reset();
        // Setup vision rectangle with specified image pattern
        this.recrec.setup("legos");
        waitForStart();
        //telemetry.addData("time setup recrec", timer.seconds());

        // Grab the corners, and their average X value, of the image
        timer.reset();
        Mat output = new Mat();
        while (output.empty()) {
            if (hasNewFrame()) {
                output = this.recrec.processFrame(getFrameRgba(), getFrameGray());
                discardFrame();
            }
        }
        telemetry.addData("jesus process", timer.seconds());
        timer.reset();

        Point p1 = new Point(output.get(0, 0));
        Point p2 = new Point(output.get(1, 0));
        Point p3 = new Point(output.get(2, 0));
        Point p4 = new Point(output.get(3, 0));
        double avgX = (p1.x + p2.x + p3.x + p4.x) / 4.0;
        double move = (avgX-450)/200;
        if (move > 1) {
            move = 1;
        } else if (move < -1) {
            move = -1;
        } else if (move < .1 && move > -.1) {
            move = 0;
        }
        telemetry.addData("time get frame and corners", timer.seconds() );
        telemetry.addData("Move: ", move);
        //drive.yComp = move;
        //while (drive.driveToPosition(1000,  Math.abs(move)) && opModeIsActive()) {}
    }
}
