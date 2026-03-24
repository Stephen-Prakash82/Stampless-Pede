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
import swervelib.SwerveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
        // CommandScheduler.getInstance().schedule(c_moveToDistance);
        poseTag = m_vision.getTagPose(VisionConstants.ktargetTagIDs);
        System.out.println("e");
        double yaw;
        System.out.println(m_swerve.getHeading().getDegrees());
        Optional<Double> targetOptional = m_vision.getTargetTagYaw(10);
        if (targetOptional.isPresent()) {
            yaw = targetOptional.get();
            System.out.println(yaw);
            m_swerve.drive(m_swerve.getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(-yaw)));
        }
    }
    

    //private Rotation2d generateCompensatedVector() {

        // double idealSpeed = ShooterSubsystem.distanceToVelocityMap.get(currentDistance).exitVelocityMPS();

        //double yaw = m_vision.getTargetTagYaw(VisionConstants.ktargetTagIDs);

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

        //return yaw;
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
    //}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double yaw;
        // Optional<Double> targetOptional = m_vision.getTargetTagYaw(10);
        // if (targetOptional.isPresent()) {
        //     yaw = targetOptional.get();
        //     m_swerve.drive(m_swerve.getTargetSpeeds(m_controller.getLeftX(),
        //                         m_controller.getLeftY(),
        //                         Rotation2d.fromDegrees(-yaw)));
        // }
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