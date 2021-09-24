package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmScissors;
import frc.robot.commands.ClimberManual;
import frc.robot.commands.ConveyorAutomated;
import frc.robot.commands.DriveManual;
import frc.robot.commands.LightsController;
import frc.robot.commands.Scissors;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Safety;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ScissorRunner;

public class RobotContainer {
  // Subsystems
  private final Base m_base = new Base();
  private final Safety m_safety = new Safety();
  private final Collector m_collector = new Collector();
  private final ScissorRunner m_scissorRunner = new ScissorRunner();
  private final Conveyor m_conveyor = new Conveyor();
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private final Lights m_lights = new Lights();

  // Auto command setups, per WPI example
  SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();
  // SendableChooser<Double> m_scissorSpeed = new SendableChooser<Double>();

  // CAN IDs
  public static int baseRightFrontCANID = 40; // Brushless
  public static int baseRightBackCANID = 10; // Brushless
  public static int baseLeftFrontCANID = 12; // Brushless
  public static int baseLeftBackCANID = 41; // Brushless
  public static int collectorRollerCANID = 20; // Brushless
  public static int conveyorBeltCANID = 12; // Brushless
  public static int shooterRotateCANID = 60; // Brushed
  public static int shooterFlywheelCANID = 61; // Brushless
  public static int climberLeftCANID = 21; // Brushless
  public static int climberRightCANID = 10; // Brushless
  public static int panelSpinnerCANID = 30; // Brushed

  // Solenoid Ports
  public static int collectorSolPort = 1;
  public static int climberLeftSolPort = 0;

  // PWM Ports
  public static int shooterLeftServoPort = 0;
  public static int shooterRightServoPort = 1;
  public static int lightsPWMPort = 2;

  // DIO Ports
  public static int breakbeam1DIOPort = 9;
  public static int breakbeam2DIOPort = 8;
  public static int breakbeam3DIOPort = 5;
  public static int climberLeftLowLimitSwitch = 6;
  public static int climberRightLowLimitSwitch = 7;

  // Controller Ports
  Joystick mainJS = new Joystick(0);

  public RobotContainer() {
    configureDriverStation();
    configureBaseController();
    configurePassiveCommands();
    configureAutoChooser();
  }

  private void configureDriverStation() {
    SmartDashboard.putNumber("Scissor Speed (Percent)", 30);
  }

  private void configureBaseController() {
    SmartDashboard.putNumber("axis's", mainJS.getAxisCount());

    // make sure that on init the scissors are unarmed
    // m_safety.armScissor(false);

    new JoystickButton(mainJS, 4).whileHeld(new ClimberManual(m_climber, () -> 1, () -> 1));
    new JoystickButton(mainJS, 2).whileHeld(new ClimberManual(m_climber, () -> -1, () -> -1));

    new JoystickButton(mainJS, 6).whenPressed(new ArmScissors(m_safety, m_base, m_scissorRunner));
    // new JoystickButton(mainJS, 8).whenPressed(new Scissors(m_scissorRunner, () ->
    // getScissorSelectedSpeed(), m_safety));

    m_base.setDefaultCommand(new DriveManual(m_base, () -> mainJS.getRawAxis(1), () -> mainJS.getRawAxis(3)));
  }

  private double getScissorSelectedSpeed() {
    double _speed = SmartDashboard.getNumber("Scissor Speed (Percent)", 30);

    return _speed / 100;
  }

  private void configurePassiveCommands() {
    m_conveyor.setDefaultCommand(new ConveyorAutomated(m_conveyor));
    m_lights.setDefaultCommand(new LightsController(m_lights, m_conveyor));
  }

  private void configureAutoChooser() {
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}