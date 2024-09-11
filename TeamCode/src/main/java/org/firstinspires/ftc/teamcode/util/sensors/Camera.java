package org.firstinspires.ftc.teamcode.util.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Camera {
    private final AprilTagProcessor aprilTag;
    private final OpenCvCamera cameraUSBguyMAX9000;
    private final ColorDetectionPipeline colorDetectionPipeline;

    // Constructor to initialize the camera and pipelines
    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Get the webcam name from the hardware map
        WebcamName webcamName = hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        // Create the camera object
        cameraUSBguyMAX9000 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Initialize the color detection pipeline
        colorDetectionPipeline = new ColorDetectionPipeline();
        cameraUSBguyMAX9000.setPipeline(colorDetectionPipeline);

        // Open the camera device asynchronously
        cameraUSBguyMAX9000.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming the camera feed
                cameraUSBguyMAX9000.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera errors
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });
    }

    // Method to get AprilTag detections
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    // Method to get the camera status
    public String getStatus() {
        return cameraUSBguyMAX9000.getFps() > 0 ? "STREAMING" : "NOT STREAMING";
    }

    // Method to get the detected color as a string
    public String getColor() {
        return colorDetectionPipeline.getDetectedColor();
    }

    // Inner class for the color detection pipeline
    class ColorDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "UNKNOWN"; // Variable to store the detected color

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the region of interest (center 25% of the frame)
            int centerX = input.cols() / 2;
            int centerY = input.rows() / 2;
            int regionWidth = input.cols() / 4;
            int regionHeight = input.rows() / 4;
            Rect reccyBoi = new Rect(centerX - regionWidth / 2, centerY - regionHeight / 2, regionWidth, regionHeight);
            Mat centerMat = hsv.submat(reccyBoi);

            // Determine the color in the center region
            if (isColorInRange(centerMat, new Scalar(20, 150, 70), new Scalar(30, 255, 255))) {
                detectedColor = "YELLOW";
            } else if (isColorInRange(centerMat, new Scalar(0, 150, 70), new Scalar(10, 255, 255)) ||
                       isColorInRange(centerMat, new Scalar(170, 150, 70), new Scalar(180, 255, 255))) {
                detectedColor = "RED";
            } else if (isColorInRange(centerMat, new Scalar(100, 150, 70), new Scalar(140, 255, 255))) {
                detectedColor = "BLUE";
            } else {
                detectedColor = "UNKNOWN";
            }

            // Draw rectangles around the detected colors for visualization
            Scalar drawColor = new Scalar(255, 0, 0); // Default to blue
            if (detectedColor.equals("RED")) {
                drawColor = new Scalar(0, 0, 255);
            } else if (detectedColor.equals("YELLOW")) {
                drawColor = new Scalar(0, 255, 255);
            }
            Imgproc.rectangle(input, reccyBoi, drawColor, 2);

            return input;
        }

        // Method to get the detected color
        public String getDetectedColor() {
            return detectedColor;
        }

        // Helper method to check if a color is within a specified range
        private boolean isColorInRange(Mat mat, Scalar lowerBound, Scalar upperBound) {
            Mat mask = new Mat();
            Core.inRange(mat, lowerBound, upperBound, mask);
            double nonZeroCount = Core.countNonZero(mask);
            double totalPixels = mat.total();
            return (nonZeroCount / totalPixels) > 0.5; // Threshold to determine if the color is present
        }
    }
}