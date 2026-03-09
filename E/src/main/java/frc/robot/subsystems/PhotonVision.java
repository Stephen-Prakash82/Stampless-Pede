package frc.robot.subsystems;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("camera1");// Change this to match the name of your camera
    //@Override
    public void teleopPeriodic() {
    //     // Calculate drivetrain commands from Joystick values do not know why or how this exists in thier example code
    //     double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
    //     double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
    //     double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;

        // Read in relevant data from the Camera
        boolean targetVisible = false;
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
                        targetVisible = true;
                    }
                }
            }
        }
    }
    public Command alignWithTag() {
        return runOnce(()-> {
            //add code to turn
        });
    }
}