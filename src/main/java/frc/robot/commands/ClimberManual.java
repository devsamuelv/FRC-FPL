package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberManual extends CommandBase {
  private final Climber m_climber;
  private DoubleSupplier rightSpeed;

  public ClimberManual(Climber subsystem, DoubleSupplier rightSpeed) {
    this.m_climber = subsystem;
    this.rightSpeed = rightSpeed;

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
    SmartDashboard.putNumber("Climber Left Encoder Value", m_climber.getClimberPos());
    SmartDashboard.putNumber("Climber Right Encoder Value", m_climber.getClimberPos());
    m_climber.setArmed(true);

    if (m_climber.getRightLowLimitSwitch())
      m_climber.resetRightClimber();
    if (m_climber.getArmed()) {
      m_climber.setSolenoids(Constants.climbersUnlocked);

      m_climber.driveClimbers(0, rightSpeed.getAsDouble());

      // if (m_climber.getClimberPos() == 208) {
      // m_climber.setClimberPos(0);
      // } else if (m_climber.getClimberPos() <= 2) {
      // m_climber.setClimberPos(208);
      // }
    }
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