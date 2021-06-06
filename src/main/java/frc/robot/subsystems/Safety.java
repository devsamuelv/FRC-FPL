// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Safety extends SubsystemBase {
  private boolean scissorsIsArmed = false;

  /** Creates a new Safety. */
  public Safety() {
  }

  public boolean armScissor(boolean _isArmed) {
    SmartDashboard.putBoolean("isArmed", _isArmed);

    this.scissorsIsArmed = _isArmed;
    return this.scissorsIsArmed;
  }

  public boolean isScissorsArmed() {
    return this.scissorsIsArmed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
