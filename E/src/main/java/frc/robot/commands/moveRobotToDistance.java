// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveRobotToDistance extends Command {
    /** Creates a new DistanceLock. */
    private final SwerveSubsystem m_swerve;
    private final Vision m_vision;
    private double closestRadius;

    public moveRobotToDistance(SwerveSubsystem swervesystem, Vision visionsystem) {
        m_swerve = swervesystem;
        m_vision = visionsystem;
        addRequirements(m_swerve, m_vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        double tagCurrentDistance = m_vision.getTagDistance(VisionConstants.ktargetTagIDs);

        // figure out which radius is closest to the bot
        closestRadius = m_vision.findClosestRadius(OperatorConstants.kRadii, tagCurrentDistance);

        // get the distance of the point on the circle closest to the robot
        double distRobotToCircle = tagCurrentDistance - closestRadius;

        var yawOptional = m_vision.getTargetTagYaw(VisionConstants.ktargetTagIDs);

        // Make sure there is a yaw to use
        // yawOptional.ifPresent(
        //         yaw -> {
        //             // get a translation from the robot to the point
        //             Translation2d robotToPointTranslation = m_vision.robotToPoint(distRobotToCircle, yaw);

        //             // Move the robot by this translation
        //             m_swerve.drive(robotToPointTranslation, 0, false);
        //         });

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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