// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceLock extends Command {
    /** Creates a new DistanceLock. */

    private final SwerveSubsystem swerve;
    private final Vision vision;
    private final CommandXboxController controller;
    private double currentDistance;
    private double xTranslation;
    private double xDesiredPose;
    private double yDesiredPose;
    private double yTranslation;
    private Translation2d distLockTranslation;
    private Pose2d poseTag;

    private final int[] targetTagIDs;

    public DistanceLock(SwerveSubsystem swerveSubsystem, Vision visionsystem, CommandXboxController DriveController,
            int[] targetTagIDsArg) {
        swerve = swerveSubsystem;
        vision = visionsystem;
        controller = DriveController;
        targetTagIDs = targetTagIDsArg;
        addRequirements(swerve, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // find distance from bot to tag
        currentDistance = Vision.getTagDistance(targetTagIDs[0]);
        poseTag = Vision.getTagPose(targetTagIDs[0]);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Uses the equation of a circle with radius currentDistance and center at the
        // location of the tag to find the y translation for a joystick x-translation
        yTranslation = -1 * controller.getLeftY() * swerve.getSwerveDrive().getMaximumChassisVelocity()
                * OperatorConstants.kScale;
        yDesiredPose = Vision.robotPose.getY() + yTranslation;
        xDesiredPose = (-1 * Math.sqrt(
                currentDistance * currentDistance - (yDesiredPose - poseTag.getY()) * (yDesiredPose - poseTag.getY())))
                + poseTag.getX();
        xTranslation = xDesiredPose - Vision.robotPose.getX();

        distLockTranslation = new Translation2d(xTranslation, yTranslation);

        swerve.drive(distLockTranslation, 0, true);

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
