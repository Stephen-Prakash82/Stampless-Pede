// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // some of these must be manually added to physicalproperties.json.
  // we also need values for steering angular velocity and friction
  public static final double ROBOT_MASS = Units.lbsToKilograms(90.4); // 32lbs * kg per pound
  public static final double ROBOT_WIDTH = Units.inchesToMeters(26);
  public static final double ROBOT_LENGTH = Units.inchesToMeters(26);
  public static final double MOI = (1 / 12) * ROBOT_MASS * (ROBOT_LENGTH * ROBOT_LENGTH) * (ROBOT_WIDTH * ROBOT_WIDTH); // moment
                                                                                                                        // of
                                                                                                                        // inertia
  public static final Matter CHASSIS = new Matter(
      new Translation3d(Units.inchesToMeters(26), Units.inchesToMeters(26), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static double MAX_SPEED = 4.9;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.1;
    public static final double kScale = 0.8;
    public static final double[] kRadii = { 1, 2, 3 };
  }

  public static class DriveConstants {
    // Swerve CanIDs
    public static final int kFrontLeftDrivingCanID = 2;
    public static final int kFrontRightDrivingCanID = 4;
    public static final int kRearRightDrivingCanID = 6;
    public static final int kRearLeftDrivingCanID = 8;

    public static final int kFrontLeftTurningCanID = 1;
    public static final int kFrontRightTurningCanID = 3;
    public static final int kRearRightTurningCanID = 5;
    public static final int kRearLeftTurningCanID = 7;
  }

  // CanIDs that are negative need to be adjusted once bot is wired

  public static class IntakeConstants {
    public static final int kDeployMotorCanID = 14;
    public static final int kIntakeMotorCanID = 13;
    public static final double kArmDegreesPerRotation = 180.0; // Assuming 1 rotation equals 360 degrees, adjust if //
                                                               // necessary
    public static final double kIntakeArmRotationOffset = 95;
    public static final double kIntakeSpeed = 1; // Adjust as necessary
    public static final double kDeployDutyCycle = 0.50; // Adjust as necessary
    public static final double kRetractDutyCycle = -0.67; // Adjust as necessary

    public static final double kDeployMotorkS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kDeployMotorkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kDeployMotorkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kDeployMotorkP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kDeployMotorkI = 0; // no output for integrated error
    public static final double kDeployMotorkD = 0.1;

    public static final double kIntakeMotorkS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kIntakeMotorkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kIntakeMotorkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kIntakeMotorkP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kIntakeMotorkI = 0; // no output for integrated error
    public static final double kIntakeMotorkD = 0.1;

    public static final double kDeployMaxCurrent = 40;
    public static final double kIntakeMaxCurrent = 40;
    public static final double kDeployVelocity = 1;
    public static final double kIntakeVelocity = 10;
    public static final double kRetractVelocity = -(10);
    public static final double kIntakeJiggleVelocity = 1;
  }

  public static class ShooterConstants {
    // we lowk need dose encodors plsplspls stevenplspls
    public static final int kShooterLoaderMotorCanID = 9;
    public static final int kShooterRearMotorCanID = 10;
    public static final int kShooterFrontLowerMotorCanID = 12;
    public static final int kShooterFrontUpperMotorCanID = 11;
    public static final double kP = 1;
    public static final double kLoaderDutyCycle = 1; // Adjust as necessary
    public static final double kFrontMotorsDutyCycle = 1; // Adjust as necessary
    public static final double kRearMotorDutyCycle = .6; // Adjust as necessary
    
    public static final double kDoubleMotorkS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kDoubleMotorkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kDoubleMotorkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kDoubleMotorkP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kDoubleMotorkI = 0; // no output for integrated error
    public static final double kDoubleMotorkD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    public static final double kRearMotorkS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kRearMotorkV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    public static final double kRearMotorkA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kRearMotorkP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kRearMotorkI = 0; // no output for integrated error
    public static final double kRearMotorkD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    public static final double kFrontMotorsVelocity = 10;
    public static final double kRearMotorVelocity = 10;
    public static final double kMaxCurrent = 40; // maximum applied current (amperes)
  }

  public static class VisionConstants {

    public static final String kCameraName = "Camera1";
    // Offset from the center of the camera to the robot center
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0, 0.3937, -0.4699),
        new Rotation3d(Units.degreesToRadians(86), 0, 0));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final double kDistanceArg = -1; // TBD through testing
    public static final double kDistanceFromTarget = -1; // TBD through testing, make it a function and move it
                                                         // elsewhere
  }

  public static class GameConstants {
    // Field Layouts
    public static final AprilTagFieldLayout kAndymarkLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final AprilTagFieldLayout kWeldedLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public static class AutoConstants {
    // anyone who asks about ts is a shoot on sight
  }
}