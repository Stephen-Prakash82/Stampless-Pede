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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public static final boolean blueAlliance = DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Blue;
  public static final Matter CHASSIS = new Matter(
      new Translation3d(Units.inchesToMeters(26), Units.inchesToMeters(26), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static double MAX_SPEED = 4.9;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
    public static final double kScale = 0.8;
    public static final double[] kRadii = { 1, 2, 3 }; // as meters
    public static final double kAutoAimScale = .6;
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
    public static final int kIntakeMotor1CanID = 13;
    public static final int kHopperMotorCanID = 20;
    public static final double kArmDegreesPerRotation = 180.0; // Assuming 1 rotation equals 360 degrees, adjust if //
                                                               // necessary
    public static final double kArmToDeployRotor = 2;
    public static final double kIntakeArmRotationOffset = 95;
    public static final double kHopperDutyCycle = .5; // Adjust as necessary
    // public static final double kIntakeDutyCycle = .5;

    public static final double kDeployMotorkP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kDeployMotorkI = 0; // no output for integrated error
    public static final double kDeployMotorkD = 0.1;

    public static final double kIntakeMotor1kS = 0.25; // Add 0.25 V output to overcome static friction
    public static final double kIntakeMotor1kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    public static final double kIntakeMotor1kI = 0; // no output for integrated error
    public static final double kIntakeMotor1kD = 0.1;

    public static final double kDeployMaxCurrent = 40;
    public static final double kIntakeMaxCurrent = 40;
    public static final double kDeployPosition = 0;
    public static final double kIntakeVelocity = 100 / 60;
    public static final double kRetractPosition = kIntakeArmRotationOffset;
    public static final double kIntakeJigglePosition = 15;
  }

  public static record MotorOutputVelocities(double FrontMotorVelocity, double RearMotorVelocity) {
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

    public static final double kDoubleMotorkS = 0.185; // Add 0.25 V output to overcome static friction
    public static final double kDoubleMotorkP = .22; // A position error of 2.5 rotations results in 12 V output
    public static final double kDoubleMotorkI = 0; // no output for integrated error
    public static final double kDoubleMotorkD = 0.01; // A velocity error of 1 rps results in 0.1 V output
    public static final double kDoubleMotorkV = .103;

    public static final double kRearMotorkV = .11;
    public static final double kRearMotorkS = 0.22; // Add 0.25 V output to overcome static friction
    public static final double kRearMotorkP = .24; // A position error of 2.5 rotations results in 12 V output
    public static final double kRearMotorkI = 0; // no output for integrated error
    public static final double kRearMotorkD = 0.01; // A velocity error of 1 rps results in 0.1 V output

    public static final double kFrontMotorsVelocity = 4000;
    public static final double kRearMotorVelocity = 5000;
    public static final double kMaxVoltage = 12.5; // maximum applied current (amperes)
    public static final double kFrontRotorToRoller = 2;
    public static final double kRearRotorToRoller = 44 / 16;
  }

  public static class VisionConstants {

    public static final String[] kCameraNames = { "Camera 1", "Camera 2" };
    public static final Transform3d[] kCameraOffsets = {
        new Transform3d(Units.inchesToMeters(-10.7755), Units.inchesToMeters(6.2376), Units.inchesToMeters(20.5609),
            new Rotation3d(0, Units.degreesToRadians(26.0313), Units.degreesToRadians(25))),
        new Transform3d(Units.inchesToMeters(-10.7768), Units.inchesToMeters(-6.2376), Units.inchesToMeters(20.5609),
            new Rotation3d(0, Units.degreesToRadians(26.0313), -Units.degreesToRadians(25))) };

    // Offset from the center of the camera to the robot center

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final int[] ktargetTagIDs = blueAlliance ? new int[] { 25, 26 } : new int[] { 9, 10 };
    public static final Translation2d kMiddleHubTagOffset = blueAlliance
        ? new Translation2d(Units.inchesToMeters(23.5), 0)
        : new Translation2d(Units.inchesToMeters(-23.5), 0);
  }

  public static class GameConstants {
    // Field Layouts
    public static final AprilTagFieldLayout kAndymarkLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final AprilTagFieldLayout kWeldedLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);
    public static final AprilTagFieldLayout kFieldLayout = kWeldedLayout; // Change this constant to the field layout we
                                                                          // are using
  }

  public static class AutoConstants {
    // anyone who asks about ts is a shoot on sight
  }
}