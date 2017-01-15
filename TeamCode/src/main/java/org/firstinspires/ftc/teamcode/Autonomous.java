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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomusRed", group="autonomous")  // @Autonomous(...) is the other common choice
public class Autonomous extends LinearVisionOpMode {

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

        //initializes servo
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

        drive.setValues(1, -1, 0);
        while (drive.driveToPosition(10550, .9) && opModeIsActive()) {}

        //moves in to get camera in better location
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(500, .5) && opModeIsActive()) {}

        drive.setValues(0, 0, 0);

        //senses beacon color and moves to that side
        drive.xComp = 0;
        boolean done = false;
        String s;
        double pos = TeleOpOmni.BUTTON_MIDDLE; //the position to set the button pusher to
        while (!done && opModeIsActive()) {
            s = beacon.getAnalysis().getColorString();
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
        while (drive.driveToPosition(1600, .5) && opModeIsActive()) {}

        //adjusts the button pusher
        servoButton.setPosition(pos);
        sleep(900);
        servoButton.setPosition(TeleOpOmni.BUTTON_MIDDLE);

        //moves forwards to press button
        drive.setValues(-1, 0, 0);
        while (drive.driveToPosition(3000, .7) && opModeIsActive()) {}

        //turns toward center goal
        drive.setValues(0, 0, 1);
        while (drive.driveToPosition(4300, .4) && opModeIsActive()) {}

        //fires gun
        motorGun1.setPower(1);
        sleep(3000);
        motorGun1.setPower(0);

        drive.reset(0);

        //moves to hit cap ball
        drive.setValues(1, 0, 0);
        while (drive.driveToPosition(1500, .9) && opModeIsActive()) {}

        //rotates to pull cap ball off base plate
        drive.setValues(0, 0, 1);
        while (drive.driveToPosition(5000, .9) && opModeIsActive()) {}
    }
}
