package frc.robot.subsystems;


import org.photonvision.*;
import org.photonvision.common.*;
import org.photonvision.targeting.*;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.PhotonPoseEstimator.*;

//From docs
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

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

    //Create a pose estimator
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(GameConstants.kWeldedLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
    
    private Matrix<N3, N1> curStdDevs;

    //Initialize vision estimation variable
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    


    //Get the robot's pose on the field and distance data
    public Command getRobotFieldData() {
        return run(() -> {
            
            //Make a fallback if there aren't multiple tags
            photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            //Get all unread results
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            //Check if the latest result has April Tags
            if (!results.isEmpty()) {
                PhotonPipelineResult result = results.get(results.size() - 1); //Takes the last item in the results list, i.e. last detected result

                boolean HasTargets = result.hasTargets();
                
                //Only if there are targets
                if (HasTargets) {
                    
                    Optional<EstimatedRobotPose> visionEst = Optional.empty();
                    for (var change : camera.getAllUnreadResults()) {
                        visionEst = photonEstimator.update(change);
                        updateEstimationStdDevs(visionEst, change.getTargets());
                        
                        visionEst.ifPresent(
                            est -> {
                                // Change our trust in the measurement based on the tags we can see
                                var estStdDevs = getEstimationStdDevs();
                                var robotPose = est.estimatedPose.toPose2d();
                                var poseTimestamp = est.timestampSeconds;
                                
                            });
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
