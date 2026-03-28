package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
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
        addRequirements(m_shooter, m_intakeArm);
    }

    @Override
    public void initialize() {
        double targetDistance = m_vision.getTagDistance(VisionConstants.ktargetTagIDs[1]);
        m_shooter.runShooter(ShooterSubsystem.distanceToVelocityMap.get(.5).FrontMotorVelocityRPM(),
                getRearVelocity(targetDistance));
        m_intakeArm.runHopper();
    }

    @Override
    public void execute() {
        // Timer.delay(1);
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
        m_intakeArm.stopDeployMotor();
        m_intakeArm.stopHopper();
        m_shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getRearVelocity(double targetDistance) {
        double RearMotorVelocityRPM = 0;
        for (Map.Entry<Double, ShooterSubsystem.MotorOutputVelocities> hashMapEntry : ShooterSubsystem.distanceToVelocityMap
                .entrySet()) {
            double testDistance = hashMapEntry.getKey();
            if (testDistance > targetDistance) {
                double testLowDistance = testDistance - .25;
                double RearVelocityHigh = hashMapEntry.getValue().RearMotorVelocityRPM();
                double RearVelocityLow = ShooterSubsystem.distanceToVelocityMap.get(testLowDistance)
                        .RearMotorVelocityRPM();
                RearMotorVelocityRPM = ((RearVelocityHigh - RearVelocityLow) / (testDistance - testLowDistance))
                        * (targetDistance - testLowDistance) + RearVelocityLow;
                return RearMotorVelocityRPM;
            }
        }
        return 0;
    }
}