package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import java.util.HashMap;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.GameConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;

public class Vision extends SubsystemBase {

    public static record cameraStorageObject(Transform3d cameraOffset, PhotonCamera cameraObject,
            PhotonPoseEstimator cameraPoseEstimator) {
    };

    public static HashMap<String, cameraStorageObject> cameraHashMap = new HashMap<>();

    private final SwerveSubsystem m_swerve;

    // Choose the field layout
    private final AprilTagFieldLayout fieldLayout = GameConstants.kFieldLayout;

    // Variables for Std. Dev
    private Matrix<N3, N1> estStdDevs;
    private Matrix<N3, N1> curStdDevs;

    // Place the corresponding camera values in the cameraHashMap
    public Vision(SwerveSubsystem swerveDrive) {

        // Create the swerveDrive object
        m_swerve = swerveDrive;

        // Add values to the hashmap for each defined camera name
        for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
            cameraHashMap.put(VisionConstants.kCameraNames[i],
                    new cameraStorageObject(VisionConstants.kCameraOffsets[i],
                            new PhotonCamera(VisionConstants.kCameraNames[i]),
                            new PhotonPoseEstimator(fieldLayout, VisionConstants.kCameraOffsets[i])));

        }

    }

    // Get the robot's pose on the field and distance data
    public void getVisionPose() {

        for (Map.Entry<String, cameraStorageObject> entry : cameraHashMap.entrySet()) {

            // Get the cameraStorageObject at the i-th row of the hashmap
            cameraStorageObject cameraKey = entry.getValue();

            // Get the pose-estimator and camera stored in the cameraKey
            PhotonCamera camera = cameraKey.cameraObject;
            PhotonPoseEstimator photonEstimator = cameraKey.cameraPoseEstimator;

            // Get all unread results
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            // Check if the latest result has April Tags
            if (!results.isEmpty()) {

                for (PhotonPipelineResult result : results) {
                    // Only if results have targets
                    if (result.hasTargets()) {

                        // Make vision estimate use multiple tags
                        Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);

                        // Create a fallback if there aren't multiple tags
                        if (visionEst.isEmpty()) {
                            visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                        }

                        updateEstimationStdDevs(visionEst, result.getTargets(), photonEstimator);

                        visionEst.ifPresent(
                                est -> {
                                    // Change our trust in the measurement based on the tags we can see
                                    Matrix<N3, N1> estStdDevs = getEstimationStdDevs();
                                    Pose2d robotPose = est.estimatedPose.toPose2d();
                                    double poseTimestamp = est.timestampSeconds;

                                    m_swerve.swerveDrive.addVisionMeasurement(robotPose, poseTimestamp, estStdDevs);

                                });
                    }

                }

            }
        }

    }

    // USE THIS FUNCTION TO GET THE MOST ACCURATE ROBOT POSE!
    public Pose2d getRobotPose() {
        getVisionPose();
        return m_swerve.getPose();
    }

    // MAKE SURE TO ONLY USE VAILD TAG-IDs!!!!!!!!
    public Pose2d getTagPose(int tagID) {
        return fieldLayout.getTagPose(tagID).get().toPose2d();
    }

    public double getTagDistance(int tagID) {
        return PhotonUtils.getDistanceToPose(getRobotPose(), getTagPose(tagID));
    }

    // Get the yaw to a target
    // Uses an optional in case we cannot acquire the yaw to the target
    public Optional<Rotation2d> getTargetTagYaw(int tagID) {

        for (Map.Entry<String, cameraStorageObject> hashMapEntry : cameraHashMap.entrySet()) {

            // Get the cameraStorageObject at the i-th row of the hashmap
            cameraStorageObject cameraKey = hashMapEntry.getValue();

            // Get the camera from the cameraStorageObject
            PhotonCamera camera = cameraKey.cameraObject;

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
                        if (target.getFiducialId() == tagID) {
                            // Found tag, record its information
                            targetIntYaw = target.getYaw();
                            Rotation2d targetYaw = Rotation2d.fromRadians(targetIntYaw);

                            // Exit once yaw has been acquired
                            return Optional.of(targetYaw);
                        }
                    }
                }
            }
        }
        return Optional.empty();
    }

    public Translation2d robotToPoint(double distanceToPoint, Rotation2d yawToPoint) {
        return PhotonUtils.estimateCameraToTargetTranslation(distanceToPoint, yawToPoint);
    }

    public double findClosestRadius(double[] radii, double distance) {

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
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator poseEstimator) {
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
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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

    private Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @Override
    public void periodic() {
        getVisionPose();
    }

}
