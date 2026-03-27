package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PlaySong extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSystem m_intake;
    private final Orchestra m_Orchestra = new Orchestra();
    private StatusCode status;

    public PlaySong(ShooterSubsystem shooter, IntakeSystem intake) {
        m_shooter = shooter;
        m_intake = intake;
        m_Orchestra.addInstrument(m_intake.m_DeployMotor, 0);
        m_Orchestra.addInstrument(m_intake.m_IntakeMotor1, 0);
        m_Orchestra.addInstrument(m_intake.m_hopperMotor, 0);
        m_Orchestra.addInstrument(m_shooter.m_RearMotor, 0);
        m_Orchestra.addInstrument(m_shooter.m_FrontUpperMotor, 0);
        m_Orchestra.addInstrument(m_shooter.m_FrontLowerMotor, 0);
    }
    @Override
    public void initialize() {
        status = m_Orchestra.loadMusic("masterofpuppets.chrp");
        if (!status.isOK()) {
            System.out.println(status.value);
        }
        m_Orchestra.play();
    }
    @Override
    public void end(boolean interrupted) {
        m_Orchestra.stop();
    } // e
}
