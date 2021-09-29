package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberManual extends CommandBase {
  private final Climber m_climber;
  private BooleanSupplier isClimberUp;

  public ClimberManual(Climber subsystem, BooleanSupplier isClimberUp) {
    m_climber = subsystem;
    this.isClimberUp = isClimberUp;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Climber Right Low Switch", m_climber.getRightLowLimitSwitch());
    SmartDashboard.putNumber("Climber Left Encoder Value", m_climber.getClimberPos(Constants.leftClimber));
    SmartDashboard.putNumber("Climber Right Encoder Value", m_climber.getClimberPos(Constants.rightClimber));

    // if (m_climber.getRightLowLimitSwitch())
    // m_climber.resetRightClimber();
    // if (m_climber.getArmed()) {
    // m_climber.setSolenoids(Constants.climbersUnlocked);

    if (isClimberUp.getAsBoolean()) {
      m_climber.setClimberPos(1);
    } else if (!isClimberUp.getAsBoolean()) {
      m_climber.setClimberPos(0);
    }

    // if (leftSpeed.getAsDouble() > 0.2 || rightSpeed.getAsDouble() > 0.2 ||
    // leftSpeed.getAsDouble() < -0.2
    // || rightSpeed.getAsDouble() < -0.2) {
    // m_climber.driveClimbers(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    // } else
    // m_climber.driveClimbers(0, 0);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.driveClimbers(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}