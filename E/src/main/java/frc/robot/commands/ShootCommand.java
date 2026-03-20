package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeArm m_intakeArm;
    private final Vision m_vision;
    private final int targetTagID;
    private double closestRadius;

    record MotorOutputVelocities(double FrontMotorVelocity, double RearMotorVelocity) {
    }

    private HashMap<Double, MotorOutputVelocities> distanceToVelocityMap = new HashMap<>(); // this is a placeholder for
                                                                                            // the actual map that will
                                                                                            // be used to convert
                                                                                            // distance to velocity

    public ShootCommand(ShooterSubsystem shooterArg, IntakeArm intakeArmArg, Vision visionArg, int targetTagIDArg) {
        m_shooter = shooterArg;
        m_intakeArm = intakeArmArg;
        m_vision = visionArg;
        targetTagID = targetTagIDArg;
        addRequirements(m_shooter, m_vision);
        distanceToVelocityMap.put(OperatorConstants.kRadii[0], new MotorOutputVelocities(10.0, 10.0)); // Example
                                                                                                       // mapping,
                                                                                                       // replace with
                                                                                                       // actual values
        distanceToVelocityMap.put(OperatorConstants.kRadii[1], new MotorOutputVelocities(20.0, 20.0)); // Example
                                                                                                       // mapping,
                                                                                                       // replace with
                                                                                                       // actual values
        distanceToVelocityMap.put(OperatorConstants.kRadii[2], new MotorOutputVelocities(30.0, 30.0)); // Example
                                                                                                       // mapping,
                                                                                                       // replace with
                                                                                                       // actual values
    }

    @Override
    public void initialize() {
        closestRadius = Vision.findClosestRadius(OperatorConstants.kRadii, Vision.getTagDistance(targetTagID));
        m_shooter.runFrontMotors(distanceToVelocityMap.get(closestRadius).FrontMotorVelocity());
        m_shooter.runRearMotor(distanceToVelocityMap.get(closestRadius).RearMotorVelocity());
        Timer.delay(.1);
        m_intakeArm.runIntake(-(IntakeConstants.kIntakeVelocity / 2)); // FOR NOW
        m_shooter.runLoaderMotor();
    }

    @Override
    public void execute() {
        Timer.delay(1);
        m_intakeArm.retractIntake(IntakeConstants.kIntakeJiggleVelocity);
        Timer.delay(.1);
        m_intakeArm.deployIntake(IntakeConstants.kIntakeJiggleVelocity);
        Timer.delay(.1);
        m_intakeArm.retractIntake(IntakeConstants.kIntakeJiggleVelocity);
        Timer.delay(.1);
        m_intakeArm.deployIntake(IntakeConstants.kIntakeJiggleVelocity);
        Timer.delay(.1);
        m_intakeArm.stopDeployMotor();
    }

    @Override
    public void end(boolean isFinished) {
        m_intakeArm.stopIntake();
        m_shooter.stopShooter();
        m_intakeArm.stopDeployMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}