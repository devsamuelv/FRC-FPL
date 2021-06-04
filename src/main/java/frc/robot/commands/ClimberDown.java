// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberDown extends CommandBase {
  private final Climber m_climber;
  private boolean done = false;
  private int climbPos = 0;

  /** Creates a new ClimberDown. */
  public ClimberDown(Climber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_climber = subsystem;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setArmed(true);
    m_climber.setSolenoids(Constants.climbersUnlocked);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = m_climber.setClimberPosDown(12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.driveClimbers(0, 0);
    m_climber.setArmed(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
