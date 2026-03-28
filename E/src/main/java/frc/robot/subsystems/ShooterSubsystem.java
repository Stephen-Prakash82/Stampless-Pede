package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.HashMap;

class DataEntry {
    TalonFX motor;
    String name;
    double rpm;

    public DataEntry(TalonFX motor, String name, double rpm) {
        this.motor = motor;
        this.name = name;
        this.rpm = rpm;
    }

}

// FINISH MOTION MAGIC BEFORE USING THIS CODE
public class ShooterSubsystem extends SubsystemBase {
    public static record MotorOutputVelocities(double FrontMotorVelocityRPM, double RearMotorVelocityRPM/*
                                                                                                         * ,
                                                                                                         * double
                                                                                                         * exitVelocityMPS
                                                                                                         */) {
    }

    // in init function
    private final SparkMax m_LoaderMotor = new SparkMax(ShooterConstants.kShooterLoaderMotorCanID,
            MotorType.kBrushless);
    public final TalonFX m_RearMotor = new TalonFX(ShooterConstants.kShooterRearMotorCanID);
    public final TalonFX m_FrontUpperMotor = new TalonFX(ShooterConstants.kShooterFrontUpperMotorCanID);
    public final TalonFX m_FrontLowerMotor = new TalonFX(ShooterConstants.kShooterFrontLowerMotorCanID);
    private final DataEntry[] talons = new DataEntry[3];
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    public static final HashMap<Double, MotorOutputVelocities> distanceToVelocityMap = new HashMap<>();

    public ShooterSubsystem() {
        // dumb shoot distanceToVelocityMap.put(0.0, new MotorOutputVelocities(6000.0,
        // 2900.0)); // THESE
        // NEED
        distanceToVelocityMap.put(.5, new MotorOutputVelocities(6000.0, 00));// THESE
                                                                             // NEED
        // ADJUSTMENT
        distanceToVelocityMap.put(.75, new MotorOutputVelocities(6000.0, 00));// THESE
        distanceToVelocityMap.put(1.0, new MotorOutputVelocities(6000.0, 00));// THESE
        distanceToVelocityMap.put(1.25, new MotorOutputVelocities(6000.0, 00));// THESE NEED
        distanceToVelocityMap.put(1.5, new MotorOutputVelocities(6000.0, 00));// THESE
        distanceToVelocityMap.put(2.0, new MotorOutputVelocities(6000.0, 00));// THESE
        // Initialize your shooter motors and any necessary components here
        talons[0] = new DataEntry(m_RearMotor, "Rear Talon", 0);
        talons[1] = new DataEntry(m_FrontUpperMotor, "Front Upper Talon", 0);
        talons[2] = new DataEntry(m_FrontLowerMotor, "Front Lower Talon", 0);
        // The left motor is CCW+
        // in init function
        final TalonFXConfiguration rearConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive));
        rearConfig.Voltage.withPeakForwardVoltage(Volts.of(ShooterConstants.kMaxVoltage))
                .withPeakReverseVoltage(Volts.of(-ShooterConstants.kMaxVoltage));
        final TalonFXConfiguration upperConfig = rearConfig.clone()
                .withMotorOutput(rearConfig.MotorOutput.clone().withInverted(InvertedValue.CounterClockwise_Positive));
        // set slot 0 gains
        upperConfig.Slot0.kS = ShooterConstants.kDoubleMotorkS; // Add 0.25 V output to overcome static friction
        upperConfig.Slot0.kP = ShooterConstants.kDoubleMotorkP; // A position error of 2.5 rotations results in 12V
        upperConfig.Slot0.kI = ShooterConstants.kDoubleMotorkI; // no output for integrated error
        upperConfig.Slot0.kD = ShooterConstants.kDoubleMotorkD; // A velocity error of 1 rps results in 0.1 V output
        upperConfig.Slot0.kV = ShooterConstants.kDoubleMotorkV; // Add 0.25 V output to overcome static friction
        upperConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.kFrontRotorToRoller);
        // set slot 1 gains

        rearConfig.Slot0.kS = ShooterConstants.kRearMotorkS; // Add 0.25 V output to overcome static friction
        rearConfig.Slot0.kP = ShooterConstants.kRearMotorkP; // A position error of 2.5 rotations results in 12 V output
        rearConfig.Slot0.kI = ShooterConstants.kRearMotorkI; // no output for integrated error
        rearConfig.Slot0.kD = ShooterConstants.kRearMotorkD; // A velocity error of 1 rps results in 0.1 V output
        rearConfig.Slot0.kV = ShooterConstants.kRearMotorkV; // Add 0.25 V output to overcome static friction
        rearConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.kRearRotorToRoller);

        m_FrontLowerMotor.setControl(new Follower(m_FrontUpperMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        m_FrontUpperMotor.getConfigurator().apply(upperConfig.Slot0);
        m_FrontLowerMotor.getConfigurator().apply(upperConfig.Slot0);
        m_RearMotor.getConfigurator().apply(rearConfig.Slot0);
    }

    public void runLoaderMotor() {
        m_LoaderMotor.set(ShooterConstants.kLoaderDutyCycle);
    }

    public void runRearMotor(double velocity) {
        m_RearMotor.setControl(m_velocityVoltage.withVelocity(velocity / 60));
    }

    public void runFrontMotors(double velocity) {
        m_FrontUpperMotor.setControl(m_velocityVoltage.withVelocity(velocity / 60));
    }

    public Command runShooter(double front, double back) {
        return runOnce(() -> {
            runRearMotor(back);
            runFrontMotors(front);
            Timer.delay(.5);
            runLoaderMotor();
        }).finallyDo((interrupted) -> {
            stopShooter();
        });
    }

    public void stopShooter() {
        m_LoaderMotor.stopMotor();
        m_FrontUpperMotor.stopMotor();
        m_FrontLowerMotor.stopMotor();
        m_RearMotor.stopMotor();
    }

    public Command stop() {
        return runOnce(() -> {
            stopShooter();
        });
    }

    @Override
    public void periodic() {
        for (var i = 0; i < talons.length; i++) {
            talons[i].motor.getVelocity().refresh();
            double rps = talons[i].motor.getVelocity().getValueAsDouble();
            double rpm = rps * 60;
            SmartDashboard.putNumber(talons[i].name, rpm);
        }

    }
}
