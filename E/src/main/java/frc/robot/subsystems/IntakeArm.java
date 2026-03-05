package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;


public class IntakeArm extends SubsystemBase
 {
    public IntakeArm() {
        // Initialize your intake arm components here
    }
    //to do: switch to a closed loop control of the deploy motor using the angle measurement as feedback, and add a command to move to a specific angle
    
    final TalonFX m_deploymotor = new TalonFX(IntakeConstants.kDeployMotorCanID);
    final SparkMax m_intakemotor = new SparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    
    public final MutAngle mut_angle = Degrees.mutable(0);
    private final DutyCycleOut m_DeployDutyCycle = new DutyCycleOut(IntakeConstants.kDeployDutyCycle);
    private final DutyCycleOut m_RetractDutyCycle = new DutyCycleOut(IntakeConstants.kRetractDutyCycle);
    
    public Command deployIntake() {

        return run(()->{
            m_deploymotor.setControl(m_DeployDutyCycle);
        });

    }
    public Command retractIntake() {

        return run(()->{
            m_deploymotor.setControl(m_RetractDutyCycle);
        });

    }
    public Command runIntake() {
        return run(() -> {
            m_intakemotor.set(IntakeConstants.kIntakeSpeed);
        });
    }
    public Command stopIntake() {
        return run(() -> {
            m_intakemotor.set(0);
        });
    }
    public Angle getAngle() {
        double rawAngle = m_deploymotor.getPosition().getValueAsDouble() * IntakeConstants.kArmDegreesPerRotation;
        mut_angle.mut_replace(rawAngle, Degrees);
        return mut_angle;
    }
    @Override
    public void periodic() {
        this.getAngle();
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        this.getAngle(); // Update the angle measurement in simulation
    // This method will be called once per scheduler run during simulation
    }
}
