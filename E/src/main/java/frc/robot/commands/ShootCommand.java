package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSystem m_intakeArm;
    private final Vision m_vision;

    public ShootCommand(ShooterSubsystem shooterArg, IntakeSystem intakeArmArg, Vision visionArg) {
        m_shooter = shooterArg;
        m_intakeArm = intakeArmArg;
        m_vision = visionArg;
        addRequirements(m_shooter, m_vision);
    }

    @Override
    public void initialize() {
        // closestRadius = m_vision.findClosestRadius(OperatorConstants.kRadii,
        // m_vision.getTagDistance(VisionConstants.ktargetTagIDs[1]));
        m_shooter.runFrontMotors(ShooterSubsystem.distanceToVelocityMap.get(0.0).FrontMotorVelocityRPM());
        m_shooter.runRearMotor(ShooterSubsystem.distanceToVelocityMap.get(0.0).RearMotorVelocityRPM());
        Timer.delay(.1);
        m_intakeArm.runhopper();
        m_shooter.runLoaderMotor();
    }

    @Override
    public void execute() {
        //Timer.delay(1);
        // m_intakeArm.deployIntake(IntakeConstants.kRetractDutyCycle);
        // Timer.delay(.3);
        // m_intakeArm.deployIntake(IntakeConstants.kDeployDutyCycle);
        // Timer.delay(.3);
        // m_intakeArm.deployIntake(IntakeConstants.kRetractDutyCycle);
        // Timer.delay(.3);
        // m_intakeArm.deployIntake(IntakeConstants.kDeployDutyCycle);
        // Timer.delay(.3);
        // m_intakeArm.stopDeployMotor();
    }

    @Override
    public void end(boolean isFinished) {
        m_intakeArm.deployIntake(0);
        m_intakeArm.stophopper();
        m_shooter.stopShooter();
        m_intakeArm.stopDeployMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}