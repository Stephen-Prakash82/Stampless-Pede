// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ShootCommand;


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

  public final SwerveSubsystem m_swervedrive = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Vision m_vision = new Vision(m_swervedrive);
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSystem m_intake = new IntakeSystem();
  CommandXboxController m_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final ShootCommand c_ShootCommand = new ShootCommand(m_shooter, m_intake, m_vision);

  private final AutoAlign c_AutoAlign = new AutoAlign(m_swervedrive, m_vision, m_DriverController);

  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);
    NamedCommands.registerCommand("deployIntake", m_intake.deployIntakeCommand());
    //NamedCommands.registerCommand("shoot", m_shooter.runShooter());
    NamedCommands.registerCommand("runIntake", m_intake.runIntakeCommand());
    NamedCommands.registerCommand("stopIntake", m_intake.stopIntakeCommand());
    NamedCommands.registerCommand("AimLock", c_AutoAlign);
    NamedCommands.registerCommand("poseTest", m_vision.poseTest());

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
    m_DriverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    m_DriverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    m_DriverController.a().whileTrue(c_AutoAlign);
    m_DriverController.leftBumper().onTrue(m_intake.retractIntakeCommand()).onTrue(m_intake.stopDeployMotorCommand());
    m_DriverController.rightBumper().onTrue(m_intake.deployIntakeCommand()).onTrue(m_intake.stopDeployMotorCommand());
    m_DriverController.rightTrigger().onTrue(c_ShootCommand);
    m_DriverController.leftTrigger().onTrue(m_intake.runIntakeCommand()).onTrue(m_intake.stopIntakeCommand());
    m_DriverController.a().whileTrue(c_AutoAlign);
    m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand())
        .onFalse(driveFieldOrientedAngularVelocity);
    
    m_DriverController.povDown().onTrue(m_shooter.runShooter(6000, 2900)).onFalse(m_shooter.stop());
    m_DriverController.povUp().onTrue(m_shooter.runShooter(6000, 2600)).onFalse(m_shooter.stop());
    m_DriverController.povLeft().onTrue(m_shooter.runShooter(6000, 2700)).onFalse(m_shooter.stop());
    m_DriverController.povRight().onTrue(m_shooter.runShooter(6000, 2800)).onFalse(m_shooter.stop());

    //m_DriverController.x().onTrue(m_vision.poseTest());
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