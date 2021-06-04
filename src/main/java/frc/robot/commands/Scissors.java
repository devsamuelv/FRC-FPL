// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class Scissors extends CommandBase {
  private final Base m_base;
  private DoubleSupplier m_speed;
  private BooleanSupplier arm;
  private boolean isArmed;

  /** Creates a new Scissors. */
  public Scissors(Base _m_base, DoubleSupplier _m_speed, BooleanSupplier _isArmed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_base = _m_base;
    this.m_speed = _m_speed;
    this.arm = _isArmed;

    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getAsBoolean()) {
      isArmed = !arm.getAsBoolean();
    }

    if (this.isArmed) {
      m_base.runScissors(this.m_speed.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
