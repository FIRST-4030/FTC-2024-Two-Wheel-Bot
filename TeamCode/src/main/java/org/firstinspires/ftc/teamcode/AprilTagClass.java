/*
 * April Tag Class
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
  */
package org.firstinspires.ftc.teamcode;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class AprilTagClass {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    
    private OpMode myOpMode; // To enable using telemetry()

    private List<AprilTagDetection> currentDetections;
    
    private AprilTagDetection desiredTag = null;  // used to hold data for detected tag

    //private static final boolean USE_WEBCAM = true;  // BLACK true for webcam
    private static final boolean USE_WEBCAM = false;  // Blue false for phone camera

    /**
     * Initialize the AprilTag processor.  Constructor
     */
    public AprilTagClass(HardwareMap hardwareMap,OpMode opMode) {
        
        this.myOpMode = opMode;  // To enable using telemetry()

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            .setDrawCubeProjection(true)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(this.aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method constructor
    
    public int getDetections() {

        currentDetections = aprilTag.getDetections();

        for(AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null) {
                desiredTag = detection;  // assume only one for now
            }
        }
        
        return currentDetections.size();

    }
    /*
    public AprilTagPoseFtc[] getTranslationToTags() {
        
        for(int i=0;i<aprilTagTranslations.length;i++)
        {
            aprilTagTranslations[i] = null;
        }
        aprilTagDetections.forEach((AprilTagDetection) -> {
            aprilTagTranslations[AprilTagDetection.id] = AprilTagDetection.ftcPose;
        });
        return aprilTagTranslations;
    } 

    public ArrayList<AprilTagDetection> getAprilTagDetections () {
        return aprilTagDetections;
    } */

    public double getBearing() {
        if(desiredTag != null) {
            return desiredTag.ftcPose.bearing;
        } else {
            return 0.0;
        }
    }
    public double getRange() {
        if(desiredTag != null) {
            return desiredTag.ftcPose.range;
        } else {
            return 0.0;
        }
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null && !USE_WEBCAM) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera STREAM", "Waiting");
            myOpMode.telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            myOpMode.telemetry.addData("Camera STREAM", "Ready");
            //myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (USE_WEBCAM) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            myOpMode.telemetry.addData("Camera Exposure", exposureMS);
            myOpMode.telemetry.addData("Camera Gain", gain);
            myOpMode.telemetry.update();
        }
    }
    /*
    Method to call sleep from within a sub method (not in OpMode)
    */
    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e) {}
    }
}
