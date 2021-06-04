package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class ClimberUp extends CommandBase {
  private final Climber m_climber;
  private boolean done = false;
  private int climbPos = 0;

  public ClimberUp(Climber subsystem, int climb_pos) {
    m_climber = subsystem;
    climbPos = climb_pos;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.setArmed(true);
    m_climber.setSolenoids(Constants.climbersUnlocked);
  }

  @Override
  public void execute() {
    System.out.println("up");

    if (climbPos == 1) { // Climb To Mid
      done = m_climber.setClimberPos(Constants.climberTop);
    } else if (climbPos == 2) { // Climb To Left
      done = m_climber.setClimberPos(Constants.climberRibbon);
    } else if (climbPos == 3) { // Climb To Right
      done = m_climber.setClimberPos(Constants.climberMax);
    } else if (climbPos == 4) { // Climb
      done = m_climber.setClimberPos(m_climber.getClimberPos(Constants.rightClimber) + 50);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.driveClimbers(0, 0);
    m_climber.setArmed(true);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}