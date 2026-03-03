// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    public static final int kDeployMotorCanID = -1;
    public static final int kIntakeMotorCanID = -1;
  }

  public static class ShooterConstants{
    public static final int kShooterLoaderMotorCanID = -1;
    public static final int kShooterRearMotorCanID = -1;
    public static final int kShooterFrontLowerMotrCanID = -1;
    public static final int kShooterFrontUpperMotorCanID = -1;
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
