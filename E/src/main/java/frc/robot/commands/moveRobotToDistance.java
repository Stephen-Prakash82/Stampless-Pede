// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveRobotToDistance extends Command {
    /** Creates a new DistanceLock. */
    private final SwerveSubsystem swerve;
    private final Vision vision;

    private int[] targetTagIDs;
    private double closestRadius;

    public moveRobotToDistance(SwerveSubsystem swervesystem, Vision visionsystem, int[] targetTagIDArgs) {
        swerve = swervesystem;
        vision = visionsystem;
        targetTagIDs = targetTagIDArgs;
        addRequirements(swerve, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Get the current distance from the tag
        Rotation2d yawOfTag = Vision.getTargetTagYaw(targetTagIDs[0]);
        double tagCurrentDistance = Vision.getTagDistance(targetTagIDs[0]);

        // figure out which radius is closest to the bot
        closestRadius = Vision.findClosestRadius(OperatorConstants.kRadii, tagCurrentDistance);

        // get the distance of the point on the circle closest to the robot
        double distRobotToCircle = tagCurrentDistance - closestRadius;

        // get a translation from the robot to the point
        Translation2d robotToPointTranslation = Vision.robotToPoint(distRobotToCircle, yawOfTag);

        // Move the robot by this translation
        swerve.drive(robotToPointTranslation, 0, false);

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
