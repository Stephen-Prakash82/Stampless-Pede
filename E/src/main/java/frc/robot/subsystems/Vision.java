package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

//Constants and Extras
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.GameConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends SubsystemBase {

    // Create the camera
    private static final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);

    // Choose the field layout
    public static AprilTagFieldLayout fieldLayout = GameConstants.kAndymarkLayout;

    // Variable for Std. Dev
    private static Matrix<N3, N1> curStdDevs;

    // Initialize vision estimation variable
    private static Optional<EstimatedRobotPose> visionEst = Optional.empty();

    // Initialize the robot pose estimator
    private static PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(fieldLayout,
            VisionConstants.kRobotToCam);

    // Initialize variables for robot pose drivetrain integration
    public static Pose2d robotPose;
    public static double poseTimestamp;
    public static Matrix<N3, N1> estStdDevs;

    // Initialize the distance calculation variables
    public static double distFromAprilTagInMeters;
    public static Pose2d tagPose;
    private static Optional<Pose3d> tagPose3d;

    // Initialize the target yaw variable
    public static Rotation2d targetYaw = Rotation2d.kZero;
    // Target we want to get the distance to, change this number when we find out

    // Get the robot's pose on the field and distance data
    public static void getVisionPose() {

        // Get all unread results
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        // Check if the latest result has April Tags
        if (!results.isEmpty()) {

            for (PhotonPipelineResult result : results) {
                // Only if results have targets
                if (result.hasTargets()) {

                    // Make vision estimate use multiple tags
                    visionEst = photonEstimator.estimateCoprocMultiTagPose(result);

                    // Create a fallback if there aren't multiple tags
                    if (visionEst.isEmpty()) {
                        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                    }

                    updateEstimationStdDevs(visionEst, result.getTargets());

                    visionEst.ifPresent(
                            est -> {
                                // Change our trust in the measurement based on the tags we can see
                                var estStdDevs = getEstimationStdDevs();
                                robotPose = est.estimatedPose.toPose2d();
                                poseTimestamp = est.timestampSeconds;

                            });
                }

            }

        }

    }

    public static Pose2d getTagPose(int tagID) {

        // Get all unread results
        List<PhotonPipelineResult> tagResults = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : tagResults) {

            var targets = result.getTargets();

            for (var target : targets) {
                if (target.getFiducialId() == tagID) {
                    tagPose3d = fieldLayout.getTagPose(target.getFiducialId());

                    // Extract the Pose2d out of the Optional
                    tagPose3d.ifPresent(
                            tag -> {

                                // Turn the 3d pose into a 2d pose
                                tagPose = tag.toPose2d();

                            });
                }
            }
        }

        return tagPose;

    }

    public static double getTagDistance(int tagID) {
        distFromAprilTagInMeters = PhotonUtils.getDistanceToPose(robotPose, getTagPose(tagID));

        return distFromAprilTagInMeters;
    }

    public static Rotation2d getTargetTagYaw(int tagID) {

        // Get all unread results
        List<PhotonPipelineResult> latestResults = camera.getAllUnreadResults();
        double targetIntYaw = 0;
        // Check if the latest result has April Tags
        if (!latestResults.isEmpty()) {
            var latestResult = latestResults.get(latestResults.size() - 1);

            // Get the yaw
            if (latestResult.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : latestResult.getTargets()) {
                    if (target.getFiducialId() == tagID) {// need to replace with tag we are looking for
                        // Found Tag 0, record its information
                        targetIntYaw = target.getYaw();
                        targetYaw = Rotation2d.fromRadians(targetIntYaw);

                    }
                }
            }
        }
        return targetYaw;
    }

    public static Translation2d robotToPoint(double distanceToPoint, Rotation2d yawToPoint) {
        return PhotonUtils.estimateCameraToTargetTranslation(distanceToPoint, yawToPoint);
    }

    public static double findClosestRadius(double[] radii, double distance) {

        int closest = 0;

        for (int i = 0; i < radii.length; i++) {
            double radius = radii[i];
            if (Math.abs(radius - distance) < radii[closest]) {
                closest = i;
            }
        }

        return radii[closest];
    }

    // Std Dev Calculation from Docs:
    // https://github.com/PhotonVision/photonvision/blob/e8efef476b3b4681c8899a8720774d6dbd5ccf56/photonlib-java-examples/poseest/src/main/java/frc/robot/Robot.java
    private static void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }

    }

    private static Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

}
