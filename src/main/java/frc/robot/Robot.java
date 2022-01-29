// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.motorcontrol.Spark;
=======
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
>>>>>>> 2aa73e3e45339ce748720aabcadb10398f6010b5

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */



public class Robot extends TimedRobot {
  private DifferentialDrive robot_motors;
  
  private Joystick left_driver_controller;
  private Joystick right_driver_controller;

  private DoubleSolenoid IntakeSolenoid;
  private DoubleSolenoid EscapementSolenoid;
  private DoubleSolenoid Climber1Solenoid;
  private DoubleSolenoid Climber2Solenoid;


  private final Timer timer = new Timer();

  private static final String kDefaultAuto = "Default";
  private static final String kRightSide = "Right Side";
  private static final String kLeftSide = "Left Side";
  
  private String m_autoSelected;
  private String m_autoCargo;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private static final String kRightCargo = "Right Cargo";
  private static final String kLeftCargo = "Left Cargo";

  private final SendableChooser<String> m_cargochooser = new SendableChooser<>();

  private VictorSP left_motor_control = new VictorSP(0);
  private VictorSP right_motor_control = new VictorSP(2);
<<<<<<< HEAD

  private Spark climber_extension = new Spark(7);
=======
  private VictorSP outer_intake_motor = new VictorSP(4);
  private VictorSP inner_intake_motor = new VictorSP(5);
  private VictorSP conveyor_motor = new VictorSP(6);
  private Spark climber_motor = new Spark(7);
  private CANSparkMax shooter_motor = new CANSparkMax(28, MotorType.kBrushless);

>>>>>>> 2aa73e3e45339ce748720aabcadb10398f6010b5
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Right Side", kRightSide);
    m_chooser.addOption("Left Side", kLeftSide);
    SmartDashboard.putData("Robot Tarmac Options", m_chooser);

    m_cargochooser.addOption("Right Cargo", kRightCargo);
    m_cargochooser.addOption("Left Cargo", kLeftCargo);
    SmartDashboard.putData("Cargo Options", m_cargochooser);


    robot_motors = new DifferentialDrive(left_motor_control, right_motor_control);

    left_driver_controller= new Joystick(1);
    right_driver_controller = new Joystick(0);

    IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    EscapementSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    Climber1Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    Climber2Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoCargo = m_cargochooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    timer.reset();
    timer.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
      case kRightSide:
        if (m_autoCargo == kRightCargo){
          robot_motors.tankDrive(0.5, 0.5);
        }
        
        if (m_autoCargo == kLeftCargo){
          robot_motors.tankDrive(-0.5, -0.5);
        }
        break;
      
        case kLeftSide:
          if (m_autoCargo == kRightCargo){
            robot_motors.tankDrive(0.5, -0.5);
          }
          
          if (m_autoCargo == kRightCargo){
            robot_motors.tankDrive(-0.5, 0.5);
            if(timer.get() == 5){
              robot_motors.stopMotor();
            }
          }
        
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    robot_motors.tankDrive(left_driver_controller.getY(), right_driver_controller.getY());

    if(right_driver_controller.getRawButton(12))
    {
    climber_extension.set(.9);
    }
    else
    {
    climber_extension.set(0);
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
