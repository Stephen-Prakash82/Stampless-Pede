// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final boolean blueAlliance = DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Blue;
  private final int[] ktargetTagIDs = blueAlliance ? new int[] { 25, 26 } : new int[] { 9, 10 };
  private final SwerveSubsystem m_swervedrive = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"), blueAlliance);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeArm m_intake = new IntakeArm();
  private final Vision m_vision = new Vision();
  CommandXboxController m_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final AimLock c_AimLock = new AimLock(m_swervedrive, m_vision, ktargetTagIDs);
  private final ShootCommand c_ShootCommand = new ShootCommand(m_shooter, m_intake, m_vision, ktargetTagIDs[0]);
  private final DistanceLock c_DistLock = new DistanceLock(m_swervedrive, m_vision, m_DriverController, ktargetTagIDs);
  private final moveRobotToDistance c_MoveToDistance = new moveRobotToDistance(m_swervedrive, m_vision, ktargetTagIDs);
  private final AutoAlign c_AutoAlign = new AutoAlign(c_AimLock, c_DistLock, c_MoveToDistance);

  public RobotContainer() {

    NamedCommands.registerCommand("deployIntake", m_intake.deployIntakeCommand());
    NamedCommands.registerCommand("shoot", m_shooter.runShooter());
    NamedCommands.registerCommand("runIntake", m_intake.runIntakeCommand());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntakeCommand());
    NamedCommands.registerCommand("AimLock", c_AimLock);
    NamedCommands.registerCommand("DistanceLock", c_DistLock);

    configureBindings();
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swervedrive.getSwerveDrive(),
      () -> m_DriverController.getLeftY() * -1,
      () -> m_DriverController.getLeftX() * -1)
      .withControllerRotationAxis(m_DriverController::getRightX)
      .deadband(OperatorConstants.kDeadband)
      .scaleTranslation(OperatorConstants.kScale)
      .allianceRelativeControl(true);

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
    Command driveFieldOrientedAngularVelocity = m_swervedrive.driveFieldOriented(driveAngularVelocity);
    m_swervedrive.setDefaultCommand(driveFieldOrientedAngularVelocity);

    m_DriverController.leftBumper().onTrue(m_intake.retractIntakeCommand()).onFalse(m_intake.stopDeployMotorCommand());
    m_DriverController.rightBumper().onTrue(m_intake.deployIntakeCommand()).onFalse(m_intake.stopDeployMotorCommand());
    m_DriverController.rightTrigger().onTrue(m_shooter.runShooter()).onTrue(m_intake.runIntakeReverse())
        .onFalse(m_shooter.stop()).onFalse(m_intake.stopIntakeCommand());
    m_DriverController.leftTrigger().onTrue(m_intake.runIntakeCommand()).onFalse(m_intake.stopIntakeCommand());
    m_DriverController.a().toggleOnTrue(c_AimLock).onFalse(driveFieldOrientedAngularVelocity);
    m_DriverController.b().onTrue(m_swervedrive.centerModulesCommand())
        .onFalse(driveFieldOrientedAngularVelocity);
    m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand())
        .onFalse(driveFieldOrientedAngularVelocity);
    m_DriverController.x().toggleOnTrue(m_swervedrive.sysIdAngleMotorCommand());
    m_DriverController.y().toggleOnTrue(m_swervedrive.sysIdDriveMotorCommand());

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