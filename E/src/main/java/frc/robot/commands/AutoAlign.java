package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoAlign extends Command {

  private final AimLock c_aimLock;
  private final DistanceLock c_distLock;
  private final moveRobotToDistance c_moveToDistance;

  public AutoAlign(AimLock aimLockArg, DistanceLock distanceLock, moveRobotToDistance moveToDistance) {
    c_aimLock = aimLockArg;
    c_distLock = distanceLock;
    c_moveToDistance = moveToDistance;
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(c_moveToDistance);
    CommandScheduler.getInstance().schedule(c_aimLock);
    CommandScheduler.getInstance().schedule(c_distLock);
  }

  @Override
  public void execute() {
    CommandScheduler.getInstance().schedule(c_aimLock);
    CommandScheduler.getInstance().schedule(c_distLock);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}