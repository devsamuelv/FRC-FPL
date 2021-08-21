// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScissorRunner;
import frc.robot.subsystems.Safety;

public class Scissors extends CommandBase {
  private final ScissorRunner m_scissors;
  private final Safety m_safety;
  private DoubleSupplier m_speed;

  /** Creates a new Scissors. */
  public Scissors(ScissorRunner _scissors, DoubleSupplier _m_speed, Safety _safety) {
    // Use addRequirements() here to declare subsystem dependencies.

    SmartDashboard.putBoolean("isArmed", false);

    this.m_safety = _safety;
    this.m_scissors = _scissors;
    this.m_speed = _m_speed;

    addRequirements(m_scissors);
    addRequirements(m_safety);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_safety.isScissorsArmed()) {
      m_scissors.runScissors(this.m_speed.getAsDouble());
    }

    if (!m_safety.isScissorsArmed()) {
      m_scissors.runScissors(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_safety.isScissorsArmed();
  }
}
