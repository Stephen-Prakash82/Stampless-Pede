// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAlign extends Command {
  /** Creates a new AimLock. */
  private final SwerveSubsystem m_swerve;
  private final Vision m_vision;
  private final CommandXboxController m_controller;
  private final moveRobotToDistance c_moveToDistance;
  private Rotation2d targetYaw; // Do not use without checking if optional is present
  private double currentDistance;
  private double xTranslation;
  private double xDesiredPose;
  private double yDesiredPose;
  private double yTranslation;
  private Pose2d poseTag;

  public AutoAlign(SwerveSubsystem swervesystem, Vision visionsystem, CommandXboxController DriveController,
      moveRobotToDistance moveToDistance) {
    m_swerve = swervesystem;
    m_vision = visionsystem;
    m_controller = DriveController;
    c_moveToDistance = moveToDistance;
    addRequirements(m_swerve, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(c_moveToDistance);
    poseTag = m_vision.getTagPose(VisionConstants.ktargetTagIDs[1]);
  }

  private Optional<Translation2d> generateCompensatedVector() {

    double idealSpeed = ShooterSubsystem.distanceToVelocityMap.get(currentDistance).exitVelocityMPS();

    Optional<Rotation2d> yawOptional = m_vision.getTargetTagYaw(VisionConstants.ktargetTagIDs[1]);

    yawOptional.ifPresentOrElse(
        yaw -> targetYaw = yaw,

        () -> {
          targetYaw = Rotation2d.kZero;
        });

    Translation2d targetPosition = m_vision.robotToPoint(currentDistance, targetYaw);

    // Translation to be moved per second
    Translation2d robotVelocity = new Translation2d(Meters.of(m_swerve.getRobotVelocity().vxMetersPerSecond),
        Meters.of(m_swerve.getRobotVelocity().vyMetersPerSecond)); // Field centric velocity!

    // Calculate the ideal exit velocity magnitude (based on distance)
    Translation2d targetVector = targetPosition.div(currentDistance).times(idealSpeed);

    return (yawOptional.isPresent()) ? Optional.of(targetVector.minus(robotVelocity)) : Optional.empty();
    // double turretAngle = shotVector.getAngle().getDegrees();
    // double requiredSpeed = shotVector.getNorm();

    // OPTION 1: Variable Flywheel Speed (Fixed Hood)
    // double shooterRPM = calcRPM(requiredSpeed);

    // OPTION 2: Variable Hood Angle (Constant Speed)
    // double constantTotalSpeed = 15.0; // m/s
    // double newPitch = Math.acos(requiredSpeed / constantTotalSpeed);

    // NOTE: For high-arc shots (like Rapid React or Rebuilt), `idealSpeed` MUST be
    // the
    // horizontal component of the exit velocity, NOT the total speed (RPM).
    // If your lookup table gives total speed/RPM, you must project it:
    // idealSpeed_Horizontal = Total_Speed * cos(release_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // add code to turn bot constantly
    // Uses the equation of a circle with radius currentDistance and center at the
    // location of the tag to find the y translation for a joystick x-translation
    currentDistance = m_vision.getTagDistance(VisionConstants.ktargetTagIDs[1]);

    yTranslation = -1 * m_controller.getLeftY() * m_swerve.getSwerveDrive().getMaximumChassisVelocity()
        * OperatorConstants.kScale;
    yDesiredPose = m_vision.getRobotPose().getY() + yTranslation;
    xDesiredPose = (-1 * Math.sqrt(
        currentDistance * currentDistance - (yDesiredPose - poseTag.getY()) * (yDesiredPose - poseTag.getY())))
        + poseTag.getX();
    xTranslation = xDesiredPose - m_vision.getRobotPose().getX();
    generateCompensatedVector().ifPresent(
        yaw -> {
          var rotationVelocity = m_swerve.getTargetSpeeds(xTranslation, yTranslation, yaw.getAngle());
          m_swerve.driveFieldOriented(rotationVelocity);
        });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
