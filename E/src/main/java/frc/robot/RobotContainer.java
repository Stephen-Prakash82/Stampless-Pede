// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.commands.AutoAlign;
//import frc.robot.commands.PlaySong;
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
    private final Vision m_vision = new Vision(m_swervedrive, m_swervedrive.swerveDrive::addVisionMeasurement);
   
    private final IntakeSystem m_intake = new IntakeSystem();
     private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_intake);
    //private final PlaySong c_playsong = new PlaySong(m_shooter, m_intake);
    private final CommandXboxController m_DriverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final ShootCommand c_ShootCommand = new ShootCommand(m_shooter, m_intake, m_vision);

    private final AutoAlign c_AutoAlign = new AutoAlign(m_swervedrive, m_vision, m_DriverController);
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        SignalLogger.enableAutoLogging(false);
        //NamedCommands.registerCommand("deployIntake", m_intake.deployIntakeCommand());
        NamedCommands.registerCommand("shoot", c_ShootCommand);
        NamedCommands.registerCommand("runIntake", m_intake.runIntakeCommand());
        NamedCommands.registerCommand("AimLock", c_AutoAlign);
        NamedCommands.registerCommand("poseTest", m_vision.poseTest());

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> stream);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swervedrive.getSwerveDrive(),
            () -> m_DriverController.getLeftY(),
            () -> m_DriverController.getLeftX())
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
        m_swervedrive
                .setDefaultCommand(
                        driveFieldOrientedAngularVelocity);
        // m_DriverController.leftBumper()
        //         .whileTrue(m_intake.retractIntakeCommand()
        // );
        // m_DriverController.rightBumper()
        //         .whileTrue(m_intake.deployIntakeCommand()
        // );// .onFalse(m_intake.stopDeployMotorCommand());
        //m_DriverController.povUp().whileTrue(c_ShootCommand.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        m_DriverController.rightTrigger()
                .whileTrue(m_shooter.runShooterCommand(6000, 3600)
                                 
        );
        m_DriverController.leftTrigger().whileTrue(m_intake.runIntakeCommand());// .onFalse(m_intake.stopIntakeCommand());
        m_DriverController.a().whileTrue(c_AutoAlign);
        m_DriverController.rightStick().onTrue(m_swervedrive.zeroGyroWithAllianceCommand())
                .onFalse(driveFieldOrientedAngularVelocity);
        m_DriverController.b().onTrue(m_swervedrive.centerModulesCommand())
                .onFalse(driveFieldOrientedAngularVelocity);
        //m_DriverController.y().whileTrue(c_playsong.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // m_DriverController.x().whileTrue(m_swervedrive.poseTest());
        m_DriverController.povDown().whileTrue(m_shooter.runShooterCommand(6000, 2500));
         m_DriverController.povUp().whileTrue(m_shooter.runShooterCommand(6000, 2550));
          m_DriverController.povLeft().whileTrue(m_shooter.runShooterCommand(6000, 2650));
           m_DriverController.povRight().whileTrue(m_shooter.runShooterCommand(6000, 2750));
            m_DriverController.x().whileTrue(m_shooter.runShooterCommand(6000, 2800));
                        m_DriverController.y().whileTrue(m_shooter.runShooterCommand(6000, 2850));
    }

    /**
     * Use this to define the command that runs during autonomous.
     *
     * <p>
     * Scheduled during {@link Robot#autonomousInit()}.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}