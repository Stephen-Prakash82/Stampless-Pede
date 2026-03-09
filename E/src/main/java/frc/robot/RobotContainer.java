// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

//import frc.robot.subsystems.MotorTest;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.subsystems.Shooter;
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
  //private final MotorTest m_motorTest = new MotorTest();


  //NamedCommand.registerCommand("shooterShoot", m_shooter.shoot());
  //NamedCommands.registerCommand("shooterShoot", m_shooter.shoot());
//   private final Shooter m_shooter = new Shooter();

  // The driver's controller
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

    // Control the drive with split-stick arcade controls
    // m_swervedrive.setDefaultCommand(
    //     m_swervedrive.arcadeDriveCommand(
    //         () -> -m_swervedriverController.getLeftY(), () -> -m_swervedriverController.getRightX()));

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // once.
    // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
    // of bindings at once
    //m_DriverController.a().whileTrue(m_swervedrive.sysIdAngleMotorCommand());
    //m_DriverController.b().whileTrue(m_swervedrive.sysIdDriveMotorCommand());

    // m_DriverController.povUp()
    //    .onTrue(m_motorTest.runTestMotor());
    //  m_DriverController.povUp()
    //    .onFalse(m_motorTest.stopTestMotor());

    m_DriverController.y()
       .onChange(m_shooter.runLoaderMotor());
     m_DriverController.x().onChange(m_shooter.stop());


    // m_DriverController
    //     .x()
    //     .and(m_swervedriverController.rightBumper())
    //     .whileTrue(m_swervedrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_swervedriverController
    //     .y()
    //     .and(m_swervedriverController.rightBumper())
    //     .whileTrue(m_swervedrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Control the shooter wheel with the left trigger
//     m_shooter.setDefaultCommand(m_shooter.runShooter(m_swervedriverController::getLeftTriggerAxis));

//     m_swervedriverController
//         .a()
//         .and(m_swervedriverController.leftBumper())
//         .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//     m_swervedriverController
//         .b()
//         .and(m_swervedriverController.leftBumper())
//         .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//     m_swervedriverController
//         .x()
//         .and(m_swervedriverController.leftBumper())
//         .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
//     m_swervedriverController
//         .y()
//         .and(m_swervedriverController.leftBumper())
//         .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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