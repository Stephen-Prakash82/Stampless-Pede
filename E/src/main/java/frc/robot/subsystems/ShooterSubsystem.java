package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    //to do: everything

    // Define your shooter motors and any necessary variables here
    final SparkMax m_LoaderMotor = new SparkMax(ShooterConstants.kShooterLoaderMotorCanID, MotorType.kBrushless);
    final TalonFX m_RearMotor = new TalonFX(ShooterConstants.kShooterRearMotorCanID);
    final TalonFX m_FrontUpperMotor = new TalonFX(ShooterConstants.kShooterFrontUpperMotorCanID);
    final TalonFX m_FrontLowerMotor = new TalonFX(ShooterConstants.kShooterFrontLowerMotorCanID);
    public ShooterSubsystem() {
        // Initialize your shooter motors and any necessary components here
    }

    public Command runShooter() {
        return run(() -> {
            // Code to set shooter motors to the desired speed
        });
    }

    // Define methods to control the shooter, such as setting speed, stopping, etc.
    
}
