// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.commands.AutoAlign;
//import frc.robot.commands.ShootCommand;
import frc.robot.commands.moveRobotToDistance;

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
  //private final IntakeArm m_intake = new IntakeArm();
  CommandXboxController m_DriverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final ShootCommand c_ShootCommand = new ShootCommand(m_shooter, m_intake, m_vision);
  private final moveRobotToDistance c_MoveToDistance = new moveRobotToDistance(m_swervedrive, m_vision);
  private final AutoAlign c_AutoAlign = new AutoAlign(m_swervedrive, m_vision, m_DriverController, c_MoveToDistance);

  public RobotContainer() {
      
    // NamedCommands.registerCommand("deployIntake", m_intake.deployIntakeCommand());
    // NamedCommands.registerCommand("shoot", m_shooter.runShooter());
    // NamedCommands.registerCommand("runIntake", m_intake.runIntakeCommand());
    // NamedCommands.registerCommand("stopIntake", m_intake.stopIntakeCommand());
    NamedCommands.registerCommand("AimLock", c_AutoAlign);

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
    //m_DriverController.leftBumper().onTrue(m_intake.retractIntakeCommand()).onFalse(m_intake.stopDeployMotorCommand());
    //m_DriverController.rightBumper().onTrue(m_intake.deployIntakeCommand()).onFalse(m_intake.stopDeployMotorCommand());
    //m_DriverController.rightTrigger().onTrue(c_ShootCommand);
    // m_DriverController.leftTrigger().onTrue(m_intake.runIntakeCommand()).onFalse(m_intake.stopIntakeCommand());
    m_DriverController.a().onTrue(c_AutoAlign);
    //m_DriverController.b().onTrue(m_swervedrive.centerModulesCommand())
    //    .onFalse(driveFieldOrientedAngularVelocity);
    m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand())
        .onFalse(driveFieldOrientedAngularVelocity);
    //m_DriverController.x().toggleOnTrue(m_swervedrive.sysIdAngleMotorCommand());
    //m_DriverController.y().toggleOnTrue(m_swervedrive.sysIdDriveMotorCommand());
    // m_DriverController.povDown().whileTrue(m_shooter.FrontsysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_DriverController.povUp().whileTrue(m_shooter.FrontsysIdDynamic(SysIdRoutine.Direction.kReverse));
    // m_DriverController.povLeft().whileTrue(m_shooter.RearsysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_DriverController.povRight().whileTrue(m_shooter.RearsysIdDynamic(SysIdRoutine.Direction.kReverse));
    //m_DriverController.a().whileTrue(m_swervedrive.sysIdAngleMotorCommand());
    //m_DriverController.b().whileTrue(m_swervedrive.sysIdDriveMotorCommand());
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