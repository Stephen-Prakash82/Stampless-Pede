// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //some of these must be manually added to physicalproperties.json.
  //we also need values for steering angular velocity and friction
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(4.8);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants{

    //Swerve CanIDs
    public static final int kFrontLeftDrivingCanID = 3;
    public static final int kFrontRightDrivingCanID = 5;
    public static final int kRearRightDrivingCanID = 7;
    public static final int kRearLeftDrivingCanID = 9;
    
    public static final int kFrontLeftTurningCanID = 2;
    public static final int kFrontRightTurningCanID = 4;
    public static final int kRearRightTurningCanID = 6;
    public static final int kRearLeftTurningCanID = 8;

  }
  
  //CanIDs that are negative need to be adjusted once bot is wired
  public static class HopperConstants{
    public static final int kFrontRollerMotorCanID = -1;
    public static final int kRearRollerMotorCanID = -1;
  }
  
  public static class IntakeConstants{
    public static final int kDeployMotorCanID = 14;
    public static final int kIntakeMotorCanID = 15;
    public static final double kArmDegreesPerRotation = 360.0; // Assuming 1 rotation equals 360 degrees, adjust if necessary
    public static final double kIntakeSpeed = 0.5; // Adjust as necessary 
    public static final double kDeployDutyCycle = 0.5; // Adjust as necessary
    public static final double kRetractDutyCycle = -0.5; // Adjust as necessary
  }

  public static class ShooterConstants{
    //we lowk need dose encodors plsplspls stevenplspls
    public static final int kShooterLoaderMotorCanID = 13;
    public static final int kShooterRearMotorCanID = 10;
    public static final int kShooterFrontLowerMotorCanID = 12;
    public static final int kShooterFrontUpperMotorCanID = 11;
    public static final double kP = 1;

    }

  public static class ClimbConstants{
    //Only God :duck:ing knows
  }

  public static class GameConstants{
    //TBD
  }

  public static class AutoConstants{
    //anyone who asks about ts is a shoot on sight
  }


}