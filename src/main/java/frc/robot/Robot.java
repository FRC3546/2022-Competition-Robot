// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.cameraserver.CameraServer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.shuffleboard.*;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private DifferentialDrive drive_train;
  
  // These are the definitions of the actual joysticks
  private Joystick left_driver_controller= new Joystick(0);
  private Joystick right_driver_controller = new Joystick(1);
  private Joystick codriver_controller = new Joystick(2);
  
  // Driver left controller buttons
  private JoystickButton IntakeButton = new JoystickButton(right_driver_controller, 1);
  private JoystickButton DriveTrainReturnButton = new JoystickButton(right_driver_controller, 8);
  private JoystickButton DriveTrainInvertButton = new JoystickButton(right_driver_controller, 7);

  
  // Driver right controller buttons
  private JoystickButton GyroResetButton = new JoystickButton(left_driver_controller, 10);

  // Co-driver buttons
  private JoystickButton ShooterOnButton = new JoystickButton(codriver_controller, 6);
  private JoystickButton ShooterOffButton = new JoystickButton(codriver_controller, 4);
  private JoystickButton ConveyorForwardButton = new JoystickButton(codriver_controller, 8);
  private JoystickButton ConveyorReverseButton = new JoystickButton(codriver_controller, 12);
  private JoystickButton ConveyorStopButton = new JoystickButton(codriver_controller, 10);
  private JoystickButton ClimberTiltButton = new JoystickButton(codriver_controller, 11);
  private JoystickButton ClimberReturnButton = new JoystickButton(codriver_controller, 9);

  private DoubleSolenoid Intake_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid CargoRelease_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid Climber_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  private final Timer timer = new Timer();

  private String m_autoSelected;
  private String m_autoCargo;

  private static final String GetCargo = "Get the Cargo";
  private static final String LeaveTarmac = "Leave Tarmac";
  private static final String Nothing = "Do Nothing";
  private final SendableChooser<String> m_routines = new SendableChooser<>();
  
  private static final String WallCargo = "Wall Cargo";
  private static final String TerminalCargo = "Terminal Cargo";
  private static final String HangarCargo = "Hanger Cargo";
  private final SendableChooser<String> m_cargochooser = new SendableChooser<>();

  private VictorSP left_motor = new VictorSP(0);
  private VictorSP right_motor = new VictorSP(2);

  private VictorSP intake_motor = new VictorSP(4);
  private VictorSP conveyor_motor = new VictorSP(6);
  private Spark climber_extension = new Spark(7);
  private CANSparkMax shooter_motor = new CANSparkMax(28, MotorType.kBrushless);

  AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  final double kP = 1;


     // Methods for the Conveyor
     public void ActivateConveyor()
     {
      conveyor_motor.set(1);
    }
    
    public void DeactivateConveyor()
    {
      conveyor_motor.set(0);
    }
    
    public void ReverseConveyor()
    {
      conveyor_motor.set(-0.5);
    } 
  

    // Methods for the Shooter
    public void ActivateShooterMotor()
    {
     shooter_motor.set(1);
    }
  
    public void DeactivateShooterMotor()
    {
      shooter_motor.set(0);
    }
  

    // Methods for the Climber
    public void TiltClimber()
    {
      Climber_Solenoid.set(Value.kForward);
    }
  
    public void ReturnClimber()
    {
      Climber_Solenoid.set(Value.kReverse);
    }

  
    // Methods for the Cargo
    public void ReleaseCargo()
    {
      CargoRelease_Solenoid.set(Value.kReverse);
    }
  
    public void StopCargo()
    {
      CargoRelease_Solenoid.set(Value.kForward);
    }


    // Methods for Intake
    public void ActivateIntake()
    {
      intake_motor.set(1);
      Intake_Solenoid.set(Value.kForward);
    }

    public void DeactivateIntake()
    {
      Intake_Solenoid.set(Value.kReverse);
      intake_motor.set(0);
    }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    left_motor.setInverted(true);
    right_motor.setInverted(true);

    m_routines.setDefaultOption("Get the Cargo", GetCargo);
    m_routines.addOption("Leave Tarmac", LeaveTarmac);
    m_routines.addOption("Do nothing", Nothing);
    SmartDashboard.putData("Robot Routines", m_routines);

    m_cargochooser.addOption("Wall Cargo", WallCargo);
    m_cargochooser.addOption("Terminal Cargo", TerminalCargo);
    m_cargochooser.addOption("Hangar Cargo", HangarCargo);
    SmartDashboard.putData("Cargo Options", m_cargochooser);

    SmartDashboard.putData("Gyro", gyro);
    Shuffleboard.getTab("Example tab").add(gyro);


    drive_train = new DifferentialDrive(left_motor, right_motor);



    ReturnClimber();
    StopCargo();

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
    m_autoSelected = m_routines.getSelected();
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
      case GetCargo:
      default:
        if (m_autoCargo == TerminalCargo){

        }
        
        if (m_autoCargo == HangarCargo){

        }

        if (m_autoCargo == WallCargo){

        }
        break;

      case LeaveTarmac:{
        drive_train.tankDrive(0.5, 0.5);
        }
        break;
      
        case Nothing:{      
            // Does nothing LOL
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

    drive_train.tankDrive(left_driver_controller.getY(), right_driver_controller.getY());


      if(IntakeButton.get())
      {
        ActivateIntake();
      }
      else{
        DeactivateIntake();
      }

      if(DriveTrainInvertButton.get())
      {

      }
      
      
      if(DriveTrainReturnButton.get())
      {
        
      }

      
      if(GyroResetButton.get())
      {
        gyro.zeroYaw();
      }


      if(ShooterOnButton.get())
      {
        ActivateShooterMotor();
      }


      if(ShooterOffButton.get())
      {
        DeactivateIntake();
      }


      if(ConveyorForwardButton.get())
      {
        ActivateConveyor();
      }


      if(ConveyorReverseButton.get())
      {
        ReverseConveyor();
      }

      if(ConveyorStopButton.get())
      {
        DeactivateConveyor();
      }


      if(ClimberTiltButton.get())
      {
        TiltClimber();
      }


      if(ClimberReturnButton.get())
      {
        ReturnClimber();
      }




    // if(right_driver_controller.getRawButton(12))
    // {
    // climber_extension.set(.9);
    // }
    // else
    // {
    // climber_extension.set(0);
    // }
      if(codriver_controller.getRawButton(12))
      {
        System.out.println("button 12 pushed");
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
