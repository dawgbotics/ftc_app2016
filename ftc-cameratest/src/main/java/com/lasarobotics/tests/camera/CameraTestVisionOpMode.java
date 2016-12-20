/*
 * Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */

package com.lasarobotics.tests.camera;

import android.os.Environment;

import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.TestableVisionOpMode;
import org.lasarobotics.vision.util.color.Color;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import static org.lasarobotics.vision.android.Util.getContext;
import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Vision OpMode run by the Camera Test Activity
 * Use TestableVisionOpModes in testing apps ONLY (but you can easily convert between opmodes just by changingt t
 */
public class CameraTestVisionOpMode extends TestableVisionOpMode {

    Mat image;
    FeatureDetector fd;
    DescriptorExtractor dx;
    DescriptorMatcher dm;
    MatOfKeyPoint objectpoints;
    MatOfKeyPoint objectdescriptors;
    @Override
    public void init() {
        super.init();
        this.setFrameSize(new Size(480, 320));


        this.image = new Mat(new Size(480, 360), CvType.CV_8UC1);
        try {
            this.image = Utils.loadResource(getContext(), R.drawable.legos, CV_LOAD_IMAGE_GRAYSCALE);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.objectpoints =  new MatOfKeyPoint();
        this.fd = FeatureDetector.create(FeatureDetector.BRISK);
        this.fd.detect(this.image, objectpoints);
        this.objectdescriptors = new MatOfKeyPoint();
        this.dx = DescriptorExtractor.create(DescriptorExtractor.BRISK);
        this.dx.compute(this.image, objectpoints, objectdescriptors);
        this.dm = DescriptorMatcher.create(DescriptorMatcher.FLANNBASED);
        // Uri path = Uri.parse("android.resource://com.lasarobotics.tests.camera/" + R.);
        File dir = Environment.getExternalStorageDirectory();
        File file = new File(dir, "flann.yml");
        //File file = new File("flann.yml");
        String filePath = file.getPath();
        System.out.print(filePath);
        this.dm.read(filePath);



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
        try {
            rgba = super.frame(rgba, gray);
            gray = Color.rapidConvertRGBAToGRAY(rgba);
            MatOfKeyPoint sceneKeyPoints = new MatOfKeyPoint();
            MatOfKeyPoint sceneDescriptors = new MatOfKeyPoint();
            this.fd.detect(rgba, sceneKeyPoints);
            this.dx.compute(rgba, sceneKeyPoints, sceneDescriptors);


            List<MatOfDMatch> matches = new LinkedList<MatOfDMatch>();
            if (this.objectdescriptors.cols() == sceneDescriptors.cols()) {
                this.dm.knnMatch(this.objectdescriptors, sceneDescriptors, matches, 2);
                LinkedList<DMatch> goodmatches = new LinkedList<DMatch>();
                float nndrRatio = 0.7f;
                if (matches.size() > 0) {
                    for (int i = 0; i < matches.size(); i++) {
                        MatOfDMatch matofDMatch = matches.get(i);
                        DMatch[] dmatcharray = matofDMatch.toArray();
                        DMatch m1 = dmatcharray[0];
                        DMatch m2 = dmatcharray[1];

                        if (m1.distance <= m2.distance * nndrRatio) {
                            goodmatches.addLast(m1);
                        }

                    }
                }
                if (goodmatches.size() >= 7) {
                    List<KeyPoint> objKeypointlist = this.objectpoints.toList();
                    List<KeyPoint> scnKeypointlist = sceneKeyPoints.toList();
                    LinkedList<Point> objectPoints = new LinkedList<>();
                    LinkedList<Point> scenePoints = new LinkedList<>();
                    for (int i = 0; i < goodmatches.size(); i++) {
                        objectPoints.addLast(objKeypointlist.get(goodmatches.get(i).queryIdx).pt);
                        scenePoints.addLast(scnKeypointlist.get(goodmatches.get(i).trainIdx).pt);
                    }
                    ColorRGBA color = new ColorRGBA("#ff0000");
                    for (int j = 0; j < scenePoints.size(); j++) {

                        Drawing.drawCircle(rgba, scenePoints.get(j), 2, color);
                    }
                    MatOfPoint2f objMatOfPoint2f = new MatOfPoint2f();
                    objMatOfPoint2f.fromList(objectPoints);
                    MatOfPoint2f scnMatOfPoint2f = new MatOfPoint2f();
                    scnMatOfPoint2f.fromList(scenePoints);

                    Mat homography = Calib3d.findHomography(objMatOfPoint2f, scnMatOfPoint2f, Calib3d.RANSAC, 3);
                    Mat obj_corners = new Mat(4, 1, CvType.CV_32FC2);
                    Mat scene_corners = new Mat(4, 1, CvType.CV_32FC2);

                    obj_corners.put(0, 0, new double[]{0, 0});
                    obj_corners.put(1, 0, new double[]{this.image.cols(), 0});
                    obj_corners.put(2, 0, new double[]{this.image.cols(), this.image.rows()});
                    obj_corners.put(3, 0, new double[]{0, this.image.rows()});
                    Core.perspectiveTransform(obj_corners, scene_corners, homography);
                    Drawing.drawLine(rgba, new Point(scene_corners.get(1, 0)), new Point(scene_corners.get(2, 0)), new ColorRGBA("#FFC107"), 4);
                    Drawing.drawLine(rgba, new Point(scene_corners.get(2, 0)), new Point(scene_corners.get(3, 0)), new ColorRGBA("#FFC107"), 4);
                    Drawing.drawLine(rgba, new Point(scene_corners.get(3, 0)), new Point(scene_corners.get(0, 0)), new ColorRGBA("#FFC107"), 4);
                    Drawing.drawLine(rgba, new Point(scene_corners.get(0, 0)), new Point(scene_corners.get(1, 0)), new ColorRGBA("#FFC107"), 4);
                    //transforming object corners to screen corners
                }

            } else {
                //object not found
                return rgba;
            }
        } catch(java.lang.ArrayIndexOutOfBoundsException e){

        }


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