package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag Helper Class
 * This utility class provides easy-to-use methods for AprilTag detection
 * that can be integrated into any OpMode.
 */
public class AprilTagHelper {
    
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean isInitialized = false;

    // Configuration constants
    private static final String DEFAULT_WEBCAM_NAME = "Webcam 1";
    private static final int DEFAULT_DECIMATION = 2;

    /**
     * Initialize the AprilTag system
     * @param hardwareMap The robot's hardware map
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, DEFAULT_WEBCAM_NAME, DEFAULT_DECIMATION);
    }

    /**
     * Initialize the AprilTag system with custom settings
     * @param hardwareMap The robot's hardware map
     * @param webcamName Name of the webcam in the configuration
     * @param decimation Image decimation factor (1-8, lower = better quality, higher = faster)
     */
    public void init(HardwareMap hardwareMap, String webcamName, int decimation) {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(decimation);

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
        
        isInitialized = true;
    }

    /**
     * Get all currently detected AprilTags
     * @return List of AprilTagDetection objects
     */
    public List<AprilTagDetection> getDetections() {
        if (!isInitialized) {
            throw new IllegalStateException("AprilTagHelper not initialized. Call init() first.");
        }
        return aprilTag.getDetections();
    }

    /**
     * Get a specific AprilTag by ID
     * @param targetId The ID of the AprilTag to find
     * @return AprilTagDetection object, or null if not found
     */
    public AprilTagDetection getAprilTagById(int targetId) {
        List<AprilTagDetection> detections = getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Check if a specific AprilTag is currently detected
     * @param targetId The ID of the AprilTag to check for
     * @return true if detected, false otherwise
     */
    public boolean isTagDetected(int targetId) {
        return getAprilTagById(targetId) != null;
    }

    /**
     * Get the number of currently detected AprilTags
     * @return Number of detected tags
     */
    public int getDetectionCount() {
        return getDetections().size();
    }

    /**
     * Get the distance to a specific AprilTag
     * @param targetId The ID of the AprilTag
     * @return Distance in inches, or -1 if not detected
     */
    public double getDistanceToTag(int targetId) {
        AprilTagDetection detection = getAprilTagById(targetId);
        if (detection != null && detection.ftcPose != null) {
            return detection.ftcPose.range;
        }
        return -1;
    }

    /**
     * Get the bearing (left/right angle) to a specific AprilTag
     * @param targetId The ID of the AprilTag
     * @return Bearing in degrees, or Double.NaN if not detected
     */
    public double getBearingToTag(int targetId) {
        AprilTagDetection detection = getAprilTagById(targetId);
        if (detection != null && detection.ftcPose != null) {
            return detection.ftcPose.bearing;
        }
        return Double.NaN;
    }

    /**
     * Get the yaw (rotation) of a specific AprilTag
     * @param targetId The ID of the AprilTag
     * @return Yaw in degrees, or Double.NaN if not detected
     */
    public double getYawToTag(int targetId) {
        AprilTagDetection detection = getAprilTagById(targetId);
        if (detection != null && detection.ftcPose != null) {
            return detection.ftcPose.yaw;
        }
        return Double.NaN;
    }

    /**
     * Add AprilTag telemetry data to the driver station
     * @param telemetry The telemetry object to add data to
     */
    public void addTelemetry(Telemetry telemetry) {
        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addData(String.format("Tag %d (%s)", detection.id, detection.metadata.name),
                        String.format("Dist: %.1f in, Bear: %.0f°, Yaw: %.0f°",
                                detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
            } else {
                telemetry.addData(String.format("Tag %d", detection.id),
                        String.format("Center: (%.0f, %.0f)", detection.center.x, detection.center.y));
            }
        }
    }

    /**
     * Add detailed telemetry for a specific AprilTag
     * @param telemetry The telemetry object to add data to
     * @param targetId The ID of the AprilTag to display details for
     */
    public void addDetailedTelemetry(Telemetry telemetry, int targetId) {
        AprilTagDetection detection = getAprilTagById(targetId);
        
        if (detection != null) {
            telemetry.addData(String.format("=== AprilTag %d ===", targetId), "DETECTED");
            
            if (detection.metadata != null) {
                telemetry.addData("Name", detection.metadata.name);
                telemetry.addData("Position (XYZ)", String.format("%.1f, %.1f, %.1f in",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addData("Rotation (PRY)", String.format("%.1f°, %.1f°, %.1f°",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addData("Range/Bearing/Elevation", String.format("%.1f in, %.1f°, %.1f°",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addData("Center", String.format("(%.0f, %.0f) pixels",
                        detection.center.x, detection.center.y));
            }
        } else {
            telemetry.addData(String.format("AprilTag %d", targetId), "NOT DETECTED");
        }
    }

    /**
     * Stop the camera streaming to save CPU resources
     */
    public void stopStreaming() {
        if (isInitialized && visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Resume camera streaming
     */
    public void resumeStreaming() {
        if (isInitialized && visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Close the vision portal and free resources
     * Call this when you're done using AprilTag detection
     */
    public void close() {
        if (isInitialized && visionPortal != null) {
            visionPortal.close();
            isInitialized = false;
        }
    }

    /**
     * Check if the AprilTag system is initialized
     * @return true if initialized, false otherwise
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Set the decimation factor for image processing
     * Lower values = better detection range but slower processing
     * Higher values = faster processing but shorter detection range
     * @param decimation Value between 1-8
     */
    public void setDecimation(int decimation) {
        if (isInitialized && aprilTag != null) {
            aprilTag.setDecimation(Math.max(1, Math.min(8, decimation)));
        }
    }
}