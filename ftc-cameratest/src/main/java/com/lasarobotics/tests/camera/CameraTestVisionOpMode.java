/*
 * Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */

package com.lasarobotics.tests.camera;

import android.net.Uri;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.TestableVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.Color;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;

import java.io.IOException;

import static org.lasarobotics.vision.android.Util.getContext;
import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_COLOR;
import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Vision OpMode run by the Camera Test Activity
 * Use TestableVisionOpModes in testing apps ONLY (but you can easily convert between opmodes just by changingt t
 */
public class CameraTestVisionOpMode extends TestableVisionOpMode {

    Mat image;
    @Override
    public void init() {
        super.init();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        //this.setCamera(Cameras.SECONDARY);

        /**
         * Set the maximum frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(480, 320));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        //enableExtension(Extensions.BEACON);         //Beacon detection
        //enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        //enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        //beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        //beacon.setColorToleranceRed(0);
        //beacon.setColorToleranceBlue(0);

        /**
         * Debug drawing
         * Enable this only if you're running test app - otherwise, you should turn it off
         * (Although it doesn't harm anything if you leave it on, only slows down image processing)
         */
        //beacon.enableDebug();


        /**
         * Set the rotation parameters of the screen
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * If you have a weird phone, you can set the "zero" orientation here as well.
         *
         * For TestableVisionOpModes, changing other settings may break the app. See other examples
         * for normal OpModes.
         */
        //rotation.setIsUsingSecondaryCamera(false);
        //rotation.disableAutoRotate();
        //rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        //rotation.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        //cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        //cameraControl.setAutoExposureCompensation();

        // Finding keypoints and descriptors

        this.image = new Mat(new Size(480, 320), CvType.CV_8UC1);
        try {
            this.image = Utils.loadResource(getContext(), R.drawable.scrn, CV_LOAD_IMAGE_GRAYSCALE);
        } catch (IOException e) {
            e.printStackTrace();
        }
        MatOfKeyPoint objectpoints = new MatOfKeyPoint();
        FeatureDetector fd = FeatureDetector.create(FeatureDetector.FAST);
        fd.detect(this.image, objectpoints);
        MatOfKeyPoint objectdescriptors = new MatOfKeyPoint();
        DescriptorExtractor dx = DescriptorExtractor.create(DescriptorExtractor.ORB);
        dx.compute(this.image, objectpoints, objectdescriptors);
        Scalar newKeypointColor = new Scalar(255, 0, 0);
        Mat outputImage = new Mat(this.image.rows(), this.image.cols(), CV_LOAD_IMAGE_COLOR);
        Features2d.drawKeypoints(this.image,objectpoints, outputImage, newKeypointColor, 0);
        this.image = outputImage; 
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {
        rgba = super.frame(rgba, gray);
        gray = Color.rapidConvertRGBAToGRAY(rgba);


        //Get beacon analysi

        //Display confidence
        //Drawing.drawText(rgba, "Confidence: " + beaconAnalysis.getConfidenceString(),
          //      new Point(0, 50), 1.0f, new ColorGRAY(255));

        //Display beacon color
        //Drawing.drawText(rgba, beaconAnalysis.getColorString(),
        //        new Point(0, 8), 1.0f, new ColorGRAY(255), Drawing.Anchor.BOTTOMLEFT);

        //Display FPS
        //Drawing.drawText(rgba, "FPS: " + fps.getFPSString(), new Point(0, 24), 1.0f, new ColorRGBA("#ffffff"));

        //Display Beacon Center
        //Drawing.drawText(rgba, "Center: " + beacon.getAnalysis().getCenter().toString(), new Point(0, 78), 1.0f, new ColorRGBA("#ffffff"));

        //Display analysis method
        //Drawing.drawText(rgba, beacon.getAnalysisMethod().toString() + " Analysis",
           //     new Point(width - 300, 40), 1.0f, new ColorRGBA("#FFC107"));

        //Display rotation sensor compensation
        //Drawing.drawText(rgba, "Rot: " + rotation.getRotationCompensationAngle()
        //        + " (" + sensors.getScreenOrientation() + ")", new Point(0, 50), 1.0f, new ColorRGBA("#ffffff"), Drawing.Anchor.BOTTOMLEFT); //"#2196F3"

        return rgba;
    }
}