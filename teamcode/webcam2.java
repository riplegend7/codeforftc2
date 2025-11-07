package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="webcamA", group="Concept")
public class webcam2 extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Camera tag size in inches (replace with your tag's real size)
    private final double TAG_SIZE_INCHES = 2.0;

    // --- NEW: Camera Resolution (Replace with your webcam's actual resolution) ---
    private final double CAMERA_WIDTH_PX = 640.0; // Example: Common webcam width
    // --- NEW: Estimated Focal Length (in pixels) for your camera/resolution combo ---
    // This value is crucial for converting pixel offset to real-world distance (cm)
    // A good initial guess for a 640x480 resolution is around 578.0, but may need calibration.
    private final double FOCAL_LENGTH_PX = 578.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ... (Initialization remains the same)

        // --- Initialize AprilTag processor ---
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // --- Initialize VisionPortal with webcam ---
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Cam")) // webcam name in config
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- NEW: Calculate the horizontal center of the camera's view in pixels ---
        final double CAMERA_CENTER_X_PX = CAMERA_WIDTH_PX / 2.0;

        while (opModeIsActive()) {

            // Get all current detections
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("# Tags", detections.size());

            for (AprilTagDetection det : detections) {

                telemetry.addLine("---- Tag ID: " + det.id);

                // Center pixel
                telemetry.addData("Center (px)", "(%.1f, %.1f)", det.center.x, det.center.y);

                // --- NEW: Calculate the horizontal offset in pixels and determine the distance to the tag in inches ---
                double offsetX_px = det.center.x - CAMERA_CENTER_X_PX;
                double distanceInches = 0.0;

                if (det.robotPose != null) {
                    // Accurate pose distance (in the FTC SDK, position is in inches)
                    double x = det.robotPose.getPosition().x;
                    double y = det.robotPose.getPosition().y;
                    double z = det.robotPose.getPosition().z;
                    distanceInches = Math.sqrt(x*x + y*y + z*z);
                    telemetry.addData("Distance (pose)", "%.2f in", distanceInches);
                } else {
                    // Rough estimate using pixel width
                    double tagWidthPx = det.corners[1].x - det.corners[0].x;
                    distanceInches = (TAG_SIZE_INCHES * FOCAL_LENGTH_PX) / tagWidthPx;
                    telemetry.addData("Estimated Distance", "%.2f in", distanceInches);
                }

                // --- NEW: Convert the pixel offset (offsetX_px) to a real-world lateral distance (cm) ---
                // Formula: Offset_cm = (Offset_px * Tag_Size_cm) / Tag_Width_px
                // A better approach using focal length (if available):
                // Offset_cm = (Offset_px * Distance_to_Tag_in_cm) / Focal_Length_px

                double distanceCm = distanceInches * 2.54; // Convert distance to cm

                // Calculate the real-world lateral offset in cm
                double lateralOffsetCm = (offsetX_px * distanceCm) / FOCAL_LENGTH_PX;

                // Determine if it's Left or Right and format the output
                String direction = lateralOffsetCm < 0 ? "LEFT" : "RIGHT";
                // Use Math.abs() for the magnitude of the offset
                double offsetMagnitude = Math.abs(lateralOffsetCm);

                // --- NEW: Telemetry Output ---
                telemetry.addData("Lateral Offset (cm)", "%.2f %s", offsetMagnitude, direction);
                telemetry.addData("Distance (cm)", "%.2f", distanceCm);
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}