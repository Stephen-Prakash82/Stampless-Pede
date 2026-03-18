// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceLock extends Command {
    /** Creates a new DistanceLock. */
    private final SwerveSubsystem swerve;
    private final Vision vision;
    private final CommandXboxController controller;
    private double currentDistance;
    private final int[]  targetTagIDs;
    public DistanceLock(SwerveSubsystem swervesystem, Vision visionsystem, CommandXboxController DriveController, int[] targetTagIDsArg) {
        swerve = swervesystem;
        vision = visionsystem;
        controller = DriveController;
        targetTagIDs = targetTagIDsArg;  
        addRequirements(swerve, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // move bot to correct position!
        currentDistance = Vision.getTagDistance(targetTagIDs[0]);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //input stream for the drive command
        SwerveInputStream distLockAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                () -> -1 * Math.sqrt(currentDistance * currentDistance
                        - controller.getLeftX() * OperatorConstants.kscale * controller.getLeftX()
                                * OperatorConstants.kscale),
                () -> controller.getLeftX() * -1 * OperatorConstants.kscale)
                .deadband(OperatorConstants.DEADBAND)
                .allianceRelativeControl(true);

        //uses input stream to drive field oriented -- works like YAGSL Example, to understand, check definition of driveFieldOriented and SwerveInputStream
        //if this doesn't work, just switch to using driveCommand(translX, translY, angrotX)
        swerve.driveFieldOriented(distLockAngularVelocity);
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
