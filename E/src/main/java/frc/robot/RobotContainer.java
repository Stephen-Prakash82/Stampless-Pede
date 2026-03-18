// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase{
    public final SwerveSubsystem m_swervedrive = new SwerveSubsystem(
  new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeArm m_intake = new IntakeArm();
  CommandXboxController m_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    
    NamedCommands.registerCommand("deployIntake", m_intake.deployIntake());
    NamedCommands.registerCommand("shoot", m_shooter.runShooter());
    NamedCommands.registerCommand("runIntake", m_intake.runIntake());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntake());
    NamedCommands.registerCommand("aimAtTarget", m_swervedrive.aimAtTarget());
    
    
    configureBindings();
  }

  public void distanceLock() {
    if (SwerveSubsystem.distLock == false) {
      CommandScheduler.getInstance().schedule(m_swervedrive.moveRobotToDistance(SwerveSubsystem.optimalShootingRadius));
      SwerveSubsystem.distLock = true;
    }

    else if (SwerveSubsystem.distLock == true) {
      distLockAngularVelocity = SwerveInputStream.of(m_swervedrive.getSwerveDrive(),
        () -> -1 * Math.sqrt(SwerveSubsystem.optimalShootingRadius*SwerveSubsystem.optimalShootingRadius - m_DriverController.getLeftX()* 0.8 *m_DriverController.getLeftX() * 0.8),
        () -> m_DriverController.getLeftX() * -1 * 0.8)
        .deadband(OperatorConstants.DEADBAND)
        .allianceRelativeControl(true);
    }
  }

  public Command driveDistLock() {
    return run(() -> {
      distanceLock();
      m_swervedrive.driveFieldOriented(distLockAngularVelocity);
    });
  }


  SwerveInputStream distLockAngularVelocity;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swervedrive.getSwerveDrive(),
      () -> m_DriverController.getLeftY() * -1,
      () -> m_DriverController.getLeftX() * -1)
      .withControllerRotationAxis(m_DriverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  private final Command driveFieldOrientedAngularVelocity(SwerveInputStream driveVelocity) {
    return run(() -> {
      SwerveSubsystem.distLock = false;
      CommandScheduler.getInstance().schedule(m_swervedrive.driveFieldOriented(driveVelocity));
    });
  }

  /**
   * Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called in the robot class constructor.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */

  public void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = driveFieldOrientedAngularVelocity(driveAngularVelocity);
    m_swervedrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_DriverController.leftBumper().onTrue(m_intake.retractIntake()).onFalse(m_intake.stopDeployMotor());
    m_DriverController.rightBumper().onTrue(m_intake.deployIntake()).onFalse(m_intake.stopDeployMotor());
    m_DriverController.rightTrigger().onTrue(m_shooter.runShooter()).onTrue(m_intake.runIntakeReverse()).onFalse(m_shooter.stop()).onFalse(m_intake.stopIntake());
    m_DriverController.leftTrigger().onTrue(m_intake.runIntake()).onFalse(m_intake.stopIntake());
    m_DriverController.a().toggleOnTrue(m_swervedrive.aimAtTarget());
    m_DriverController.b().onTrue(m_swervedrive.centerModulesCommand())
        .onFalse(driveFieldOrientedAnglularVelocity);
    m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand())
        .onFalse(driveFieldOrientedAnglularVelocity);
    m_DriverController.x().toggleOnTrue(driveDistLock())
        .onFalse(driveFieldOrientedAnglularVelocity);

  } 

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>
   * Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_swervedrive.run(() -> {
    });
  }
}