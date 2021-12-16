package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Conveyor extends SubsystemBase {
  public CANSparkMax conveyorMotor = new CANSparkMax(RobotContainer.conveyorBeltCANID, MotorType.kBrushless);
  public DigitalInput breakbeam1 = new DigitalInput(RobotContainer.breakbeam1DIOPort);
  public DigitalInput breakbeam2 = new DigitalInput(RobotContainer.breakbeam2DIOPort);
  public DigitalInput breakbeam3 = new DigitalInput(RobotContainer.breakbeam3DIOPort);
  public CANEncoder conveyorEncoder = new CANEncoder(conveyorMotor);

  public Conveyor() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor Max Capacity", getMagazineCapacity());
  }

  public boolean getBreakbeam() {
    return false;
  }

  public boolean getMagazineCapacity() {
    return !breakbeam3.get();
  }

  public void setConveyorSpeed(double speed) {
    SmartDashboard.putNumber("Conveyor Velocity", conveyorEncoder.getVelocity());
    conveyorMotor.set(speed);
  }
}
