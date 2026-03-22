// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimLock extends Command {
  /** Creates a new AimLock. */
  public final SwerveSubsystem m_swerve;
  public final Vision m_vision;
  public static int[] targetTagIDs;
  private Rotation2d targetYaw;
  
  public AimLock(SwerveSubsystem swervesystem, Vision visionsystem, int[] targetTagIDsArg) {
    m_swerve = swervesystem;
    m_vision = visionsystem;

    targetTagIDs = targetTagIDsArg;
    addRequirements(m_swerve, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  private Translation2d generateCompensatedVector() {
    double currentDistance = m_vision.getTagDistance(targetTagIDs[1]);

    double idealSpeed = ShooterSubsystem.distanceToVelocityMap.get(currentDistance).exitVelocity();

    var yawOptional = m_vision.getTargetTagYaw(targetTagIDs[1]);

    yawOptional.ifPresentOrElse(
      yaw -> 
        targetYaw = yaw,
      () -> {
        //If there is no yaw value, auto-align won't happen because the yaw will be 0
        targetYaw = Rotation2d.kZero; //FIX THIS VERY IMPORTANT... MAKE generateCompensatedVector RETURN AN OPTIONAL AND THEN IN EXECUTE ONLY RUN CODE IF THE OPTIONAL IS NOT EMPTY - ENSURE THAT AIMLOCK DOESN'T HAPPEN WITHOUT AA TAG TO ALIGN TO!!!!!
      }  
    );

    Translation2d targetPosition = m_vision.robotToPoint(currentDistance, targetYaw);

    Translation2d robotVelocity = new Translation2d(Meters.of(m_swerve.getRobotVelocity().vxMetersPerSecond), Meters.of(m_swerve.getRobotVelocity().vyMetersPerSecond)); // Field centric velocity!

    // Calculate the ideal exit velocity magnitude (based on distance)
    Translation2d targetVector = targetPosition.div(currentDistance).times(idealSpeed);
    
    return targetVector.minus(robotVelocity);
    // double turretAngle = shotVector.getAngle().getDegrees();
    // double requiredSpeed = shotVector.getNorm();

    // OPTION 1: Variable Flywheel Speed (Fixed Hood)
    // double shooterRPM = calcRPM(requiredSpeed);

    // OPTION 2: Variable Hood Angle (Constant Speed)
    // double constantTotalSpeed = 15.0; // m/s
    // double newPitch = Math.acos(requiredSpeed / constantTotalSpeed);

    // NOTE: For high-arc shots (like Rapid React or Rebuilt), `idealSpeed` MUST be the
    // horizontal component of the exit velocity, NOT the total speed (RPM).
    // If your lookup table gives total speed/RPM, you must project it:
    // idealSpeed_Horizontal = Total_Speed * cos(release_angle);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // add code to turn bot constantly
    var rotationVelocity = m_swerve.getTargetSpeeds(0.0, 0.0, generateCompensatedVector().getAngle());
    m_swerve.driveFieldOriented(rotationVelocity);
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
