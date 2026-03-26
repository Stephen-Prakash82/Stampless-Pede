package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSystem extends SubsystemBase {
    private final TalonFX m_DeployMotor = new TalonFX(IntakeConstants.kDeployMotorCanID);
    private final TalonFX m_IntakeMotor1 = new TalonFX(IntakeConstants.kIntakeMotor1CanID);
    private final TalonFX m_hopperMotor = new TalonFX(IntakeConstants.kHopperMotorCanID);
    private final DutyCycleOut m_hopperDutyCycle = new DutyCycleOut(IntakeConstants.kHopperDutyCycle);
    private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public IntakeSystem() {
        // Initialize your intake arm components here
        final TalonFXConfiguration deployConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        final TalonFXConfiguration hopperConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast));
        deployConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(IntakeConstants.kDeployMaxCurrent))
                .withPeakReverseTorqueCurrent(Amps.of(-IntakeConstants.kDeployMaxCurrent));
        final TalonFXConfiguration intakeConfig = deployConfig.clone().withMotorOutput(deployConfig.MotorOutput.clone()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive));
        // intakeConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(IntakeConstants.kIntakeMaxCurrent))
        //         .withPeakReverseTorqueCurrent(Amps.of(-IntakeConstants.kIntakeMaxCurrent));
        // set slot 0 gains
        deployConfig.Slot0.kP = IntakeConstants.kDeployMotorkP; // A position error of 2.5 rotations results in 12V
        deployConfig.Slot0.kI = IntakeConstants.kDeployMotorkI; // no output for integrated error
        deployConfig.Slot0.kD = IntakeConstants.kDeployMotorkD; // A velocity error of 1 rps results in 0.1 V output
        deployConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kArmToDeployRotor);

        // intakeConfig.Slot0.kS = IntakeConstants.kIntakeMotor1kS; // Add 0.25 V output to overcome static friction
        // intakeConfig.Slot0.kP = IntakeConstants.kIntakeMotor1kP; // A position error of 2.5 rotations results in 12V
        // intakeConfig.Slot0.kI = IntakeConstants.kIntakeMotor1kI; // no output for integrated error
        // intakeConfig.Slot0.kD = IntakeConstants.kIntakeMotor1kD; // A velocity error of 1 rps results in 0.1 V output
        m_hopperMotor.getConfigurator().apply(hopperConfig.Slot0);
        m_IntakeMotor1.getConfigurator().apply(intakeConfig.Slot0);
        m_DeployMotor.getConfigurator().apply(deployConfig.Slot0);
        m_DeployMotor.setPosition(IntakeConstants.kIntakeArmRotationOffset);
    }
    // to do: switch to a closed loop control of the deploy motor using the angle
    // measurement as feedback, and add a command to move to a specific angle

    public void deployIntake(double position) {
        m_DeployMotor.setControl(m_positionTorque.withPosition(position / IntakeConstants.kArmDegreesPerRotation));
    }

    public Command deployIntakeCommand() {
        return runOnce(() -> {
            deployIntake(IntakeConstants.kDeployPosition);
        });
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> {
            deployIntake(IntakeConstants.kRetractPosition);
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

    public void runIntake(double velocity) {
        m_IntakeMotor1.setControl(m_velocityTorque.withVelocity(velocity));
    }

    public Command runIntakeCommand() {
        return runOnce(() -> {
            runIntake(IntakeConstants.kIntakeVelocity);
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
