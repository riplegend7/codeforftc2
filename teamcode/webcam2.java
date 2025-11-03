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

    @Override
    public void runOpMode() throws InterruptedException {

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

        while (opModeIsActive()) {

            // Get all current detections
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("# Tags", detections.size());

            for (AprilTagDetection det : detections) {

                telemetry.addLine("---- Tag ID: " + det.id);

                // Center pixel
                telemetry.addData("Center (px)", "(%.1f, %.1f)", det.center.x, det.center.y);

                double distanceInches;

                if (det.robotPose != null) {
                    // Accurate pose distance
                    double x = det.robotPose.getPosition().x;
                    double y = det.robotPose.getPosition().y;
                    double z = det.robotPose.getPosition().z;
                    distanceInches = Math.sqrt(x*x + y*y + z*z);
                    telemetry.addData("Distance (pose)", "%.2f in", distanceInches);
                } else {
                    // Rough estimate using pixel width
                    double tagWidthPx = det.corners[1].x - det.corners[0].x;
                    double focalLength = 578; // typical for Logitech C920
                    distanceInches = (TAG_SIZE_INCHES * focalLength) / tagWidthPx;
                    double total = distanceInches * 2.54;
                    telemetry.addData("Estimated Distance", "%.2f cm", total);
                }

            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}
