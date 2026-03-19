package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.lang.module.ModuleReader;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

//FINISH MOTION MAGIC BEFORE USING THIS CODE
public class ShooterSubsystem extends SubsystemBase {
    // in init function
    private final SparkMax m_LoaderMotor = new SparkMax(ShooterConstants.kShooterLoaderMotorCanID,
            MotorType.kBrushless);
    final TalonFX m_RearMotor = new TalonFX(ShooterConstants.kShooterRearMotorCanID);
    final TalonFX m_FrontUpperMotor = new TalonFX(ShooterConstants.kShooterFrontUpperMotorCanID);
    final TalonFX m_FrontLowerMotor = new TalonFX(ShooterConstants.kShooterFrontLowerMotorCanID);
    final DataEntry[] talons = new DataEntry[3];

    public ShooterSubsystem() {
        // Initialize your shooter motors and any necessary components here
        var currentConfigs = new MotorOutputConfigs();
        talons[0] = new DataEntry(m_RearMotor, "Rear Talon", 0);
        talons[1] = new DataEntry(m_FrontUpperMotor, "Front Upper Talon", 0);
        talons[2] = new DataEntry(m_FrontLowerMotor, "Front Lower Talon", 0);
        // The left motor is CCW+
        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterConstants.DoubleMotorkS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = ShooterConstants.DoubleMotorkV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = ShooterConstants.DoubleMotorkA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = ShooterConstants.DoubleMotorkP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = ShooterConstants.DoubleMotorkI; // no output for integrated error
        slot0Configs.kD = ShooterConstants.DoubleMotorkD; // A velocity error of 1 rps results in 0.1 V output
        
        // set slot 1 gains
        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = ShooterConstants.RearMotorkS; // Add 0.25 V output to overcome static friction
        slot1Configs.kV = ShooterConstants.RearMotorkV; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kA = ShooterConstants.RearMotorkA; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = ShooterConstants.RearMotorkP; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = ShooterConstants.RearMotorkI; // no output for integrated error
        slot1Configs.kD = ShooterConstants.RearMotorkD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        m_FrontUpperMotor.getConfigurator().apply(currentConfigs);  
        m_FrontLowerMotor.getConfigurator().apply(currentConfigs);  
        m_FrontLowerMotor.getConfigurator().apply(talonFXConfigs);  
        m_FrontUpperMotor.getConfigurator().apply(talonFXConfigs);  
        // Ensure our followers are following their respective leader
        m_FrontLowerMotor.setControl(new Follower(m_FrontUpperMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runLoaderMotor() {
        m_LoaderMotor.set(ShooterConstants.kLoaderDutyCycle);
    }

    public void runRearMotor() {
        m_RearMotor.setControl(new DutyCycleOut(ShooterConstants.kRearMotorDutyCycle));
    }

    public void runFrontMotors() {
        m_FrontUpperMotor.setControl(new DutyCycleOut(ShooterConstants.kFrontMotorsDutyCycle));
    }

    public Command runShooter() {
        return runOnce(() -> {
            runRearMotor();
            runFrontMotors();
            Timer.delay(.1);
            runLoaderMotor();
        });
    }

    public Command stop() {
        return runOnce(() -> {
            m_LoaderMotor.stopMotor();
            m_FrontUpperMotor.stopMotor();
            m_FrontLowerMotor.stopMotor();
            m_RearMotor.stopMotor();
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
