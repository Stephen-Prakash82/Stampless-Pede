// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.IntakeArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_swervedrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeArm m_intake = new IntakeArm();
  CommandXboxController m_DriverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


      public RobotContainer(){
        // NamedCommands.registerCommand("shooterShoot", m_shooter.runLoaderMotor());
        // NamedCommands.registerCommand("shooterStop", m_shooter.stop());

        // NamedCommands.registerCommand("testStart", m_motorTest.runTestMotor());
        // NamedCommands.registerCommand("testStop", m_motorTest.stopTestMotor());

        configureBindings();

      }
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swervedrive.getSwerveDrive(),
                                                                () -> m_DriverController.getLeftY() * -1,
                                                                () -> m_DriverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_DriverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called in the robot class constructor.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
      
  public void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swervedrive.driveFieldOriented(driveAngularVelocity);
      m_swervedrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_DriverController.leftBumper().onTrue(m_intake.retractIntake()).onFalse(m_intake.stopDeployMotor());
    m_DriverController.rightBumper().onTrue(m_intake.deployIntake()).onFalse(m_intake.stopDeployMotor());
    m_DriverController.rightTrigger().onTrue(m_shooter.runShooter()).onFalse(m_shooter.stop());
    //m_DriverController.leftTrigger().onTrue(m_intake.runIntake()).onFalse(m_intake.stopIntake());
    //m_DriverController.a();
    m_DriverController.leftStick().onTrue(m_swervedrive.centerModulesCommand()).onFalse(driveFieldOrientedAnglularVelocity);    
    m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand()).onFalse(driveFieldOrientedAnglularVelocity);
 }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_swervedrive.run(() -> {});
  }
}