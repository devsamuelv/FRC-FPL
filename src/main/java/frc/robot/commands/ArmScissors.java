// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Safety;
import frc.robot.subsystems.ScissorRunner;

public class ArmScissors extends CommandBase {
  private final Base m_base;
  private final Safety m_safety;
  private final ScissorRunner m_runner;

  /** Creates a new ArmScissors. */
  public ArmScissors(Safety _safety, Base _base, ScissorRunner _runner) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_safety = _safety;
    this.m_base = _base;
    this.m_runner = _runner;

    addRequirements(m_safety);
    addRequirements(m_runner);
    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Arm Scissors");

    if (!m_safety.isScissorsArmed()) {
      m_safety.armScissor(true);
    
      return;
    }
    
    if (m_safety.isScissorsArmed()) {
      m_safety.armScissor(false);
      m_runner.runScissors(0);
    
      return;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
