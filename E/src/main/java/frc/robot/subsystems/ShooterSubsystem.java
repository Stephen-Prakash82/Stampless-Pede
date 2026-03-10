package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    //to do: everything

    // Define your shooter motors and any necessary variables here
    private final SparkMax m_LoaderMotor = new SparkMax(ShooterConstants.kShooterLoaderMotorCanID, MotorType.kBrushless);

    final TalonFX m_RearMotor = new TalonFX(ShooterConstants.kShooterRearMotorCanID);
    final TalonFX m_FrontUpperMotor = new TalonFX(ShooterConstants.kShooterFrontUpperMotorCanID);
    final TalonFX m_FrontLowerMotor = new TalonFX(ShooterConstants.kShooterFrontLowerMotorCanID);
    private final TalonFX m_DeployMotor = new TalonFX(IntakeConstants.kDeployMotorCanID);
    public ShooterSubsystem() {
        // Initialize your shooter motors and any necessary components here
        var currentConfigs = new MotorOutputConfigs();

      // The left motor is CCW+
      currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
      currentConfigs.NeutralMode = NeutralModeValue.Coast;
      m_FrontUpperMotor.getConfigurator().apply(currentConfigs);
      m_FrontLowerMotor.getConfigurator().apply(currentConfigs);

      // Ensure our followers are following their respective leader
      m_FrontLowerMotor.setControl(new Follower(m_FrontUpperMotor.getDeviceID(), MotorAlignmentValue.Aligned));
      m_RearMotor.setControl(new Follower(m_DeployMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }


    
    public Command runLoaderMotor(){
        return runOnce(() -> {
            m_LoaderMotor.set(ShooterConstants.kLoaderDutyCycle);
            
            //System.out.println(this.getEncoderCount());
    });
    }

    public Command stop(){
        return runOnce(() ->{
            m_LoaderMotor.stopMotor();
            m_DeployMotor.stopMotor();
            m_FrontUpperMotor.stopMotor();
            m_FrontLowerMotor.stopMotor();
        });
    }

    public void runRearMotor(double speed) {
        m_DeployMotor.setControl(new DutyCycleOut(speed));
    }

    public void runFrontMotors(double speed) {
        m_FrontUpperMotor.setControl(new DutyCycleOut(speed));
    }

    public Command runShooter() {
        return runOnce(() -> {
            // Code to set shooter motors to the desired speed
            //runRearMotor(-0.1);
            runFrontMotors(.05);
        });
    }

 //   public static void 

    // Define methods to control the shooter, such as setting speed, stopping, etc.
    
}
