package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.lang.module.ModuleReader;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

public class ShooterSubsystem extends SubsystemBase {
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
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        m_FrontUpperMotor.getConfigurator().apply(currentConfigs);
        m_FrontLowerMotor.getConfigurator().apply(currentConfigs);

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
