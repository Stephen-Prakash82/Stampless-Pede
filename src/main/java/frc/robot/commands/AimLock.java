// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimLock extends Command {
  /** Creates a new AimLock. */
  public final SwerveSubsystem swerve;
  public final Vision vision;
  public static int[] targetTagIDs;

  public AimLock(SwerveSubsystem swervesystem, Vision visionsystem, int[] targetTagIDsArg) {
    swerve = swervesystem;
    vision = visionsystem;
    targetTagIDs = targetTagIDsArg;
    addRequirements(swerve, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // add code to turn bot constantly
    Rotation2d yawToTurn = Vision.getTargetTagYaw(targetTagIDs[0]);

    var rotationVelocity = swerve.getTargetSpeeds(0.0, 0.0, yawToTurn);
    swerve.driveFieldOriented(rotationVelocity);
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
