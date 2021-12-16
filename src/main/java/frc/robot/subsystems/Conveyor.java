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
  public DigitalInput breakbeam1 = new DigitalInput(4);
  public DigitalInput breakbeam2 = new DigitalInput(RobotContainer.breakbeam2DIOPort);
  public DigitalInput breakbeam3 = new DigitalInput(RobotContainer.breakbeam3DIOPort);
  public CANEncoder conveyorEncoder = new CANEncoder(conveyorMotor);

  public Conveyor() {
    // conveyorMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor Max Capacity", getMagazineCapacity());
    SmartDashboard.putBoolean("breakbeam 1", breakbeam1.get());
    SmartDashboard.putBoolean("breakbeam 2", breakbeam2.get());
    SmartDashboard.putBoolean("breakbeam 3", breakbeam3.get());
  }

  public boolean getBreakbeam() {
<<<<<<< HEAD
    return false;
=======
    return false;// !breakbeam1.get();
>>>>>>> 46114360536fd08cc769fc78128e68909f80eb15
  }

  public boolean getMagazineCapacity() {
    return !breakbeam3.get();
  }

  public void setConveyorSpeed(double speed) {
    SmartDashboard.putNumber("Conveyor Velocity", conveyorEncoder.getVelocity());
    conveyorMotor.set(speed);
  }
}
