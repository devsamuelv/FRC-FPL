// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScissorRunner extends SubsystemBase {
  CANSparkMax scissors = new CANSparkMax(13, MotorType.kBrushed);

  /** Creates a new scissors. */
  public ScissorRunner() {
  }

  public void runScissors(double _power) {
    this.scissors.set(_power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
