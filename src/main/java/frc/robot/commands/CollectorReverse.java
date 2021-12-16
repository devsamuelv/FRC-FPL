package frc.robot.commands;

import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CollectorReverse extends CommandBase {
  private final Collector m_collector;
  private final Conveyor m_conveyor;

  public CollectorReverse(Collector subsystem, Conveyor conveyor) {
    m_collector = subsystem;
    this.m_conveyor = conveyor;
    addRequirements(m_collector);
    addRequirements(conveyor);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_collector.setCollectorExtendSol(Constants.collectorExtended);
    m_collector.setCollectorSpeed(Constants.collectorCollectSpeed);
    m_conveyor.setConveyorSpeed(-1);
  }

  @Override
  public void end(boolean interrupted) {
    m_collector.setCollectorExtendSol(Constants.collectorRetracted);
    m_collector.setCollectorSpeed(0);
    m_conveyor.setConveyorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}