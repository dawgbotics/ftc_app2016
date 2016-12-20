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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="auto test", group="autonomous")  // @Autonomous(...) is the other common choice
public class AutoTest extends LinearVisionOpMode {

    VisionRectangle recrec;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForVisionStart();
        this.recrec = new VisionRectangle();
        this.recrec.setup("legos");

        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        waitForStart();
        int a = 0;
        Mat output;
        while(opModeIsActive()) {
            output = new Mat();
            while (output.empty()) {
                if (hasNewFrame()) {
                    a++;
                    output = this.recrec.processFrame(getFrameRgba(), getFrameGray());
                    discardFrame();
                    telemetry.addData("count: ",a);
                }
            }
            //double offset = (output.get(0,0)[0] + output.get(1,0)[0] + output.get(2,0)[0] + output.get(3,0)[0]) / 4;

            Point p =  new Point(output.get(0, 0));
            Point p1 = new Point(output.get(1,0));
            Point p2 =  new Point(output.get(2, 0));
            Point p3 = new Point(output.get(3,0));
            telemetry.addData("avg X: ",(p.x + p1.x + p2.x + p3.x) / 4.0);
        }
    }
}