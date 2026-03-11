package frc.robot.subsystems;


import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


//From docs
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
import frc.robot.Robot;
import frc.robot.Constants.GameConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class PhotonVision extends SubsystemBase {

    //Create the camera
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);

    //Choose the field layout
    AprilTagFieldLayout fieldLayout = GameConstants.kAndymarkLayout;
    
    //Variable for Std. Dev
    private Matrix<N3, N1> curStdDevs;

    //Initialize vision estimation variable
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    //Initialize the robot pose estimator
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.kRobotToCam);

    //Initialize variables for robot pose drivetrain integration
    public Pose2d robotPose;
    public double poseTimestamp;

    //Initialize the distance calculation variables
    public double distFromAprilTagInMeters;
    public Pose2d tagPose;
    private Optional<Pose3d> tagPose3d;

    //Target we want to get the distance to, change this number when we find out
    int tagID = 0;


    //Get the robot's pose on the field and distance data
    public Command getRobotFieldData() {
        return run(() -> {

            //Get all unread results
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            //Check if the latest result has April Tags
            if (!results.isEmpty()) {
                for (PhotonPipelineResult result : results) {
                    //Only if results have targets
                    if (result.hasTargets()) {
                        
                        //Make vision estimate use multiple tags
                        visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
                        
                        //Create a fallback if there aren't multiple tags
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

                             }
                        );
                        
                        var targets = result.getTargets();

                        for (var target : targets) {
                            if (target.getFiducialId() == tagID) {
                                tagPose3d = fieldLayout.getTagPose(target.getFiducialId());
                                
                                //Extract the Pose2d out of the Optional
                                tagPose3d.ifPresent(
                                    targ -> {
                                        
                                        //Turn the 3d pose into a 2d pose
                                        tagPose = targ.toPose2d();

                                    }
                                );

                                distFromAprilTagInMeters = PhotonUtils.getDistanceToPose(robotPose, tagPose);
                            }
                        }

                    }

                        


                    
                }
                
                          
            }    
        });

    }

    @Override
    public void periodic() {

        //Get the robot's pose on the field and distance data every time the scheduler runs
        CommandScheduler.getInstance().schedule(getRobotFieldData());

        //Get the yaw every time the scheduler runs
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
          // Camera processed a new frame since last
          // Get the last one in the list.
          var result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                  if (target.getFiducialId() == -0) {// need to replace with tag we are looking for
                      // Found Tag 0, record its information
                      targetYaw = target.getYaw();
                  }
              }
          }
        }
    }

    //Std Dev Calculation from Docs: https://github.com/PhotonVision/photonvision/blob/e8efef476b3b4681c8899a8720774d6dbd5ccf56/photonlib-java-examples/poseest/src/main/java/frc/robot/Robot.java
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
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
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
        
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

}
