package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSystem extends SubsystemBase {
    public final TalonFX m_DeployMotor = new TalonFX(IntakeConstants.kDeployMotorCanID);
    public final TalonFX m_IntakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanID);
    public final TalonFX m_hopperMotor = new TalonFX(IntakeConstants.kHopperMotorCanID);
    private final DutyCycleOut m_hopperDutyCycle = new DutyCycleOut(IntakeConstants.kHopperDutyCycle);

    public IntakeSystem() {
        // Initialize your intake arm components here
        final TalonFXConfiguration deployConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        final TalonFXConfiguration hopperConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast));
        final TalonFXConfiguration intakeConfig = deployConfig.clone().withMotorOutput(deployConfig.MotorOutput.clone()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive));
        m_hopperMotor.getConfigurator().apply(hopperConfig.Slot0);
        m_IntakeMotor1.getConfigurator().apply(intakeConfig.Slot0);
        m_DeployMotor.getConfigurator().apply(deployConfig.Slot0);
        m_DeployMotor.setPosition(IntakeConstants.kIntakeArmRotationOffset);
    }

    public void deployIntake(double dutycycle) {
        m_DeployMotor.setControl(new DutyCycleOut(dutycycle));
    }

    public Command deployIntakeCommand() {
        return runOnce(() -> {
            deployIntake(IntakeConstants.kDeployDutyCycle);
        });
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> {
            deployIntake(IntakeConstants.kRetractDutyCycle);
        });
    }

    public void stopDeployMotor() {
        m_DeployMotor.stopMotor();
    }

    public Command stopDeployMotorCommand() {
        return runOnce(() -> {
            stopDeployMotor();
        });
    }

    public void runIntake(double dutycycle) {
        m_IntakeMotor1.setControl(new DutyCycleOut(dutycycle));
    }

    public Command runIntakeCommand() {
        return runOnce(() -> {
            runIntake(IntakeConstants.kIntakeDutyCycle);
        });
    }

    public void runhopper() {
        m_hopperMotor.setControl(m_hopperDutyCycle);
    }

    public void stophopper() {
        m_hopperMotor.stopMotor();
    }

    public void stopIntake() {
        m_IntakeMotor1.stopMotor();
    }

    public Command stopIntakeCommand() {
        return runOnce(() -> {
            stopIntake();
        });
    }

    public double getAngle() {
        m_DeployMotor.getPosition().refresh();
        double rawAngle = m_DeployMotor.getPosition().getValueAsDouble() * IntakeConstants.kArmDegreesPerRotation /*
                                                                                                                   * 95
                                                                                                                   * degree
                                                                                                                   * intake
                                                                                                                   * offset
                                                                                                                   */;

        return rawAngle;
    }

    @Override
    public void periodic() {
        getAngle();
        m_IntakeMotor1.getVelocity().refresh();
        SmartDashboard.putNumber("Intake Angle", getAngle());
        SmartDashboard.putNumber("Intake RPM", m_IntakeMotor1.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        getAngle(); // Update the angle measurement in simulation
        m_IntakeMotor1.getVelocity().refresh();
        SmartDashboard.putNumber("Intake Angle", getAngle());
        SmartDashboard.putNumber("Intake RPM", m_IntakeMotor1.getVelocity().getValueAsDouble());
    }
}
