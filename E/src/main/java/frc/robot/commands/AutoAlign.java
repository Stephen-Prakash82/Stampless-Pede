// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAlign extends Command {
    /** Creates a new AimLock. */
    private final SwerveSubsystem m_swerve;
    private final Vision m_vision;
    private final CommandXboxController m_controller;

    public AutoAlign(SwerveSubsystem swervesystem, Vision visionsystem, CommandXboxController DriveController) {
        m_swerve = swervesystem;
        m_vision = visionsystem;
        m_controller = DriveController;
        addRequirements(m_swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // private Rotation2d generateCompensatedVector() {

    // double idealSpeed =
    // ShooterSubsystem.distanceToVelocityMap.get(currentDistance).exitVelocityMPS();

    // double yaw = m_vision.getTargetTagYaw(VisionConstants.ktargetTagIDs);

    // Translation2d targetPosition = m_vision.robotToPoint(currentDistance,
    // targetYaw).plus(VisionConstants.kMiddleHubTagOffset);

    // // Translation to be moved per second
    // Translation2d robotVelocity = new
    // Translation2d(Meters.of(m_swerve.getRobotVelocity().vxMetersPerSecond),
    // Meters.of(m_swerve.getRobotVelocity().vyMetersPerSecond)); // Field centric
    // velocity!

    // // Calculate the ideal exit velocity magnitude (based on distance)
    // Translation2d targetVector =
    // targetPosition.div(currentDistance).times(idealSpeed);

    // return yaw;
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
    // }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // // Use if we can't get the tag pose for some godforsaken reason...
        Optional<Double> targetOptional = m_vision.backupGetTargetYaw(VisionConstants.ktargetTagIDs[1]);
        Rotation2d targetRot = m_vision.getRobotPose().getRotation();
        if (targetOptional.isPresent()) {
            double yaw = targetOptional.get();

            // targetRot.plus();
            // System.out.println(targetRot);
            if (!(-1 < yaw && yaw < 1)) {
                targetRot = Rotation2d.fromDegrees(yaw + m_vision.getRobotPose().getRotation().getDegrees());
            }
        }
        m_swerve.driveFieldOriented(
                m_swerve.getTargetSpeeds(m_controller.getLeftY()  * OperatorConstants.kAutoAimScale,
                        m_controller.getLeftX()  * OperatorConstants.kAutoAimScale,
                        targetRot));

        // // COMMENT THIS IMPLEMENTATION OF AIMLOCK OUT IF USING THE BACKUP
        // double targetYaw = m_vision.getTargetYaw(VisionConstants.ktargetTagIDs[1]);
        // Rotation2d targetRot = m_vision.getRobotPose().getRotation();

        // //Create a deadband of 1 degree of yaw, meaning that if our rotation to the
        // // targetYaw is within 1 degree, we won't rotate
        // if (!(-1 < (targetYaw - m_vision.getRobotPose().getRotation().getDegrees())
        // && -1 < (targetYaw - m_vision.getRobotPose().getRotation().getDegrees()))) {
        //  targetRot = Rotation2d.fromDegrees(targetYaw);
        // }

        // m_swerve.driveFieldOriented(m_swerve.getTargetSpeeds(m_controller.getLeftY()*-1*OperatorConstants.kAutoAimScale,
        // m_controller.getLeftX()*-1 * OperatorConstants.kAutoAimScale,
        // targetRot));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(Translation2d.kZero, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}