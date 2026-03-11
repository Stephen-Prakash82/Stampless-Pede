package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeArm extends SubsystemBase
 {
    private final TalonFX m_DeployMotor = new TalonFX(IntakeConstants.kDeployMotorCanID);
    private final SparkMax m_IntakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    
    public final MutAngle mut_angle = Degrees.mutable(0);
    private final DutyCycleOut m_DeployDutyCycle = new DutyCycleOut(IntakeConstants.kDeployDutyCycle);
    private final DutyCycleOut m_RetractDutyCycle = new DutyCycleOut(IntakeConstants.kRetractDutyCycle);
    
    public IntakeArm() {
        // Initialize your intake arm components here
        m_DeployMotor.setPosition(0);
    }
    //to do: switch to a closed loop control of the deploy motor using the angle measurement as feedback, and add a command to move to a specific angle
    
    
    public Command deployIntake() {
        return runOnce(()->{
           m_DeployMotor.setControl(m_DeployDutyCycle);
        });
    }
    public Command retractIntake() {
        return runOnce(()->{
            m_DeployMotor.setControl(m_RetractDutyCycle);
        });
    }
    public Command stopDeployMotor() {
        return runOnce(()->{
            m_DeployMotor.stopMotor();
        });
    }
    public Command runIntake() {
        return runOnce(() -> {
            m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
        });
    }
    public Command stopIntake() {
        return runOnce(() -> {
            m_IntakeMotor.stopMotor();
        });
    }
    public double getAngle() 
    {
        m_DeployMotor.getPosition().refresh();
        double rawAngle = m_DeployMotor.getPosition().getValueAsDouble() * IntakeConstants.kArmDegreesPerRotation;
        return rawAngle;
    }
    @Override
    public void periodic() {
        getAngle();
        SmartDashboard.putNumber("Intake Angle", getAngle());
        SmartDashboard.putNumber("Intake RPM", m_IntakeMotor.getEncoder().getVelocity());
    // This method will be called once per scheduler rn
    }

    @Override
    public void simulationPeriodic() {
        //this.getAngle(); // Update the angle measurement in simulation
    // This method will be called once per scheduler run during simulation
    }
}
