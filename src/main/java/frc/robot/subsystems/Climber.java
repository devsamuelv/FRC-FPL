package frc.robot.subsystems;

import javax.annotation.Nullable;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  // private Solenoid leftHoldClamp = new
  // Solenoid(RobotContainer.climberLeftSolPort);

  private DigitalInput leftLowLimitSwitch = new DigitalInput(RobotContainer.climberLeftLowLimitSwitch);
  private DigitalInput rightLowLimitSwitch = new DigitalInput(RobotContainer.climberRightLowLimitSwitch);

  private CANSparkMax leftClimberMotor = new CANSparkMax(RobotContainer.climberLeftCANID, MotorType.kBrushless);
  private CANSparkMax rightClimberMotor = new CANSparkMax(RobotContainer.climberRightCANID, MotorType.kBrushless);

  // private CANEncoder leftClimberEncoder = new CANEncoder(leftClimberMotor);
  private CANEncoder rightClimberEncoder = new CANEncoder(rightClimberMotor);

  private boolean armed = true;

  public Climber() {
    rightClimberMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void resetLeftClimber() {
    // leftClimberEncoder.setPosition(0);
  }

  public void resetRightClimber() {
    rightClimberEncoder.setPosition(0);
  }

  // Drives the climber motors to a target position using P control; true = left,
  // false = right;
  public boolean setClimberPos(double right_target_position) {
    boolean rightdone = false;
    SmartDashboard.putNumber("Climber Left Encoder Value", getClimberPos());
    SmartDashboard.putNumber("Climber Right Encoder Value", getClimberPos());

    System.out.println("switch: " + rightLowLimitSwitch.get());
    System.out.println("position: " + rightClimberEncoder.getPosition());
    System.out.println("position to: " + right_target_position);

    // if (!_invert) {
    // rightClimberMotor.setInverted(true);
    // } else if (_invert) {
    // rightClimberMotor.setInverted(false);
    // }

    // System.out.println("k");

    if (rightClimberEncoder.getPosition() > right_target_position) {
      // go up
      rightClimberMotor.set(-1);
    } else {
      rightClimberMotor.set(0);
      rightdone = true;
    }

    return rightdone;
  }

  // NOT CONNECTED TO CONTROLLER
  public boolean setClimberPosDown(double right_target_position) {
    boolean rightdone = false;
    SmartDashboard.putNumber("Climber Left Encoder Value", getClimberPos());
    SmartDashboard.putNumber("Climber Right Encoder Value", getClimberPos());

    System.out.println("switch: " + rightLowLimitSwitch.get());
    System.out.println("position: " + rightClimberEncoder.getPosition() * 1.0);
    System.out.println("position to: " + right_target_position);

    if (rightClimberEncoder.getPosition() * 1.0 < right_target_position) {
      rightClimberMotor.set(1);
    } else {
      rightClimberMotor.set(0);
      rightdone = true;
    }

    return rightdone;
  }

  // USED
  // Drive the climber motors seperately at the speeds input
  public void driveClimbers(double leftSpeed, double rightSpeed) {
    rightClimberMotor.set(rightSpeed);
  }

  // Returns the encoder position of the specified climber motor; true = left,
  // false = right
  public double getClimberPos() {
    return rightClimberMotor.getEncoder().getPosition();
  }

  public boolean getRightLowLimitSwitch() {
    return rightLowLimitSwitch.get();
  }

  // NOT CONNECTED TO CONTROLLER
  public void setSolenoids(boolean pos) {
    // leftHoldClamp.set(pos);
  }

  // NOT CONNECTED TO CONTROLLER
  public void setArmed(boolean arm) {
    armed = arm;
  }

  public boolean getArmed() {
    return armed;
  }
}