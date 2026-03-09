package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    //to do: everything

    // Define your shooter motors and any necessary variables here
    private final SparkMax m_LoaderMotor = new SparkMax(ShooterConstants.kShooterLoaderMotorCanID, MotorType.kBrushless);

    final TalonFX m_RearMotor = new TalonFX(ShooterConstants.kShooterRearMotorCanID);
    final TalonFX m_FrontUpperMotor = new TalonFX(ShooterConstants.kShooterFrontUpperMotorCanID);
    final TalonFX m_FrontLowerMotor = new TalonFX(ShooterConstants.kShooterFrontLowerMotorCanID);
    public ShooterSubsystem() {
        // Initialize your shooter motors and any necessary components here
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
            
        });
    }

    public void runRearMotor(double speed) {
        m_LoaderMotor.set(speed);
    }

    public void runFrontUpperMotor(double speed) {
        m_LoaderMotor.set(speed);
    }

    public void runFrontLowerMotor(double speed) {
        m_LoaderMotor.set(speed);
    }

    public Command runShooter() {
        return run(() -> {
            // Code to set shooter motors to the desired speed

        });
    }

 //   public static void 

    // Define methods to control the shooter, such as setting speed, stopping, etc.
    
}
