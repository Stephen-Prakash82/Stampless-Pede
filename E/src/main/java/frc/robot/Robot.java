// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.AnalogInput;

// Starts recording to data log

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // private final AnalogInput m_analogInput1 = new AnalogInput(0);
  // private final AnalogEncoder m_encoder1 = new AnalogEncoder(m_analogInput1);

  // private final AnalogInput m_analogInput2 = new AnalogInput(0);
  // private final AnalogEncoder m_encoder2 = new AnalogEncoder(m_analogInput2);

  // private final AnalogInput m_analogInput3 = new AnalogInput(0);
  // private final AnalogEncoder m_encoder3 = new AnalogEncoder(m_analogInput3);

  // private final AnalogInput m_analogInput4 = new AnalogInput(0);
  // private final AnalogEncoder m_encoder4 = new AnalogEncoder(m_analogInput4);

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    DataLogManager.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // DoubleLogEntry encoderLogger1 = new DoubleLogEntry(DataLogManager.getLog(), "/my/encoders/encoder0");
    // DoubleLogEntry encoderLogger2 = new DoubleLogEntry(DataLogManager.getLog(), "/my/encoders/encoder1");
    // DoubleLogEntry encoderLogger3 = new DoubleLogEntry(DataLogManager.getLog(), "/my/encoders/encoder2");
    // DoubleLogEntry encoderLogger4 = new DoubleLogEntry(DataLogManager.getLog(), "/my/encoders/encoder3");

    // encoderLogger1.append(m_encoder1.get());
    // encoderLogger2.append(m_encoder2.get());
    // encoderLogger3.append(m_encoder3.get());
    // encoderLogger4.append(m_encoder4.get());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
