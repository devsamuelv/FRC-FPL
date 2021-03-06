package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberManual extends CommandBase {
  private final Climber m_climber;
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;

  public ClimberManual(Climber subsystem, DoubleSupplier leftStick, DoubleSupplier rightStick) {
    m_climber = subsystem;
    leftSpeed = leftStick;
    rightSpeed = rightStick;

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
    SmartDashboard.putNumber("Climber Right Commanded Value", rightSpeed.getAsDouble());
    SmartDashboard.putNumber("Climber Left Commanded Value", leftSpeed.getAsDouble());

    if (m_climber.getRightLowLimitSwitch())
      m_climber.resetRightClimber();
    if (m_climber.getArmed()) {
      m_climber.setSolenoids(Constants.climbersUnlocked);

      if (leftSpeed.getAsDouble() > 0.2 || rightSpeed.getAsDouble() > 0.2 || leftSpeed.getAsDouble() < -0.2
          || rightSpeed.getAsDouble() < -0.2) {
        m_climber.driveClimbers(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
      } else
        m_climber.driveClimbers(0, 0);
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