package frc.robot.subsystems;

public class PhotonVision {
    PhotonCamera camera = new PhotonCamera("photonvision");// Change this to match the name of your camera
    @Override
    public void teleopPeriodic() {
        // Calculate drivetrain commands from Joystick values
        double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
        double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
        double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;

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

        // Auto-align when requested
        if (controller.getAButton() && targetVisible) {//need to change button
            // Driver wants auto-alignment to tag ??
            // And, tag ?? is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            while (true) {
                turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
                //add code to turn bot constantly
                if (controller.getAButton()) {//change button
                    break
                }
            }
        } else {
            SmartDashboard.putBoolean("Is Target Found:", targetVisible)
        }
    }
}