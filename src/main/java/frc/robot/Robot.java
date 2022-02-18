
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Timer has been changed to autoTimer and reset/start
//has been moved to autonomousinit to fix timing problems while testing autonomous

/**
 * CHANGE LOG:
 * 
 * 2/18/22 BAC: Added Change Log
 */


package frc.robot;

// implements the "IterativeRobotBase program framework"
import edu.wpi.first.wpilibj.TimedRobot;

// imports for motors
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// imports for pnuematics
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// import for the Nav X gyro
import com.kauailabs.navx.frc.AHRS;

// import that is used to connect the Nav X to usb
import edu.wpi.first.wpilibj.SerialPort;

// Smartdashboard imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// joystick imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// camera server import
import edu.wpi.first.cameraserver.CameraServer;

// timer import
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  
  //creates drive train object of differential drive class
  private DifferentialDrive drive_train;
  
  // These are the definitions of the actual joysticks
  private Joystick left_driver_controller= new Joystick(0);
  private Joystick right_driver_controller = new Joystick(1);
  private Joystick codriver_controller = new Joystick(2);
  
  // Driver right controller buttons
  private JoystickButton IntakeButton = new JoystickButton(right_driver_controller, 1);
  private JoystickButton DriveTrainReturnButton = new JoystickButton(right_driver_controller, 6);
  private JoystickButton DriveTrainInvertButton = new JoystickButton(right_driver_controller, 7);
  
  // Driver left controller buttons
  private JoystickButton GyroResetButton = new JoystickButton(left_driver_controller, 10);

  // Co-driver buttons
  private JoystickButton HigherShootingSpeedButton = new JoystickButton(codriver_controller, 7);
  private JoystickButton ShooterOffButton = new JoystickButton(codriver_controller, 9);
  private JoystickButton LowerShootingSpeedButton = new JoystickButton(codriver_controller, 11);
  private JoystickButton ConveyorForwardButton = new JoystickButton(codriver_controller, 8);
  private JoystickButton ConveyorStopButton = new JoystickButton(codriver_controller, 10);
  private JoystickButton ConveyorReverseButton = new JoystickButton(codriver_controller, 12);
  private JoystickButton ClimberTiltButton = new JoystickButton(codriver_controller, 5);
  private JoystickButton ClimberReturnButton = new JoystickButton(codriver_controller, 3);
  private JoystickButton climberActivationButton = new JoystickButton(codriver_controller, 6);
  private JoystickButton climberDeactivationButton = new JoystickButton(codriver_controller, 4);

  // Creates double solenoids for future reference
  private DoubleSolenoid Intake_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid CargoRelease_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid Climber_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  // values for toggle(with seperate buttons) buttons
  private boolean isDriveTrainInverted = false;
  private boolean isClimberActivated = false;
  private boolean IntakeValue = false;
  private boolean ConveyorValue = false;
  private boolean ClimberTiltValue = false;
  private String ShooterValue = "OFF";

  // creates game timer
  private final Timer autoTimer = new Timer();

  private double intakeTimer;


  // values for which auto routine we are using used when we pull which selector we have choosen
  private String m_autoSelected;
  private String m_autoCargo;
  private String m_autoOrder;

  // chooser for primary routine(defaults as doing nothing)
  private static final String test = "test";
  private static final String depositCargoleave = "Deposit Cargo then Leave";
  private static final String GetCargo = "Get the Cargo";
  private static final String LeaveTarmac = "Leave Tarmac";
  private static final String Nothing = "Do Nothing";
  private final SendableChooser<String> m_routines = new SendableChooser<>();
  
  // chooser get cargo(determines which cargo we are going for from set positions)
  private static final String WallCargo = "Wall Cargo";
  private static final String TerminalCargo = "Terminal Cargo";
  private static final String HangarCargo = "Hanger Cargo";
  private final SendableChooser<String> m_cargochooser = new SendableChooser<>();

  // chooser for order(if we get cargo)
  private static final String DepositFirst = "Deposit First";
  private static final String FetchFirst = "Fetch Ball First";
  private final SendableChooser<String> m_order = new SendableChooser<>();

  // creates drive train victorSP motor controllers
  private VictorSP left_motor = new VictorSP(0);
  private VictorSP right_motor = new VictorSP(2);

  // creates other motor controllers
  private VictorSP intake_motor = new VictorSP(4);
  private VictorSP conveyor_motor = new VictorSP(6);
  private Spark climber_extension = new Spark(7);
  private CANSparkMax shooter_motor = new CANSparkMax(28, MotorType.kBrushless);

  // creates gyro object for navx board
  AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  final double kP = 1;

  // method for finding our toggle button values
    public void updateButtonValues()
    {
      if(DriveTrainInvertButton.get()) {
        isDriveTrainInverted = true;
      }
      else if(DriveTrainReturnButton.get())
      {
        isDriveTrainInverted = false;
      }
      if(climberActivationButton.get()) {
        isClimberActivated = true;
      }
      else if(climberDeactivationButton.get())
      {
        isClimberActivated = false;
      }
    }

    // Methods for the Conveyor
    public void ActivateConveyor()
    {
      conveyor_motor.set(1);
      ConveyorValue = true;
    }

    public void DeactivateConveyor()
    {
      conveyor_motor.set(0);
      ConveyorValue = false;
    }

    public void ReverseConveyor()
    {
      conveyor_motor.set(-0.5);
      ConveyorValue = true;
    } 


    // Methods for the Shooter
    public void ActivateShooterMotor()
    {
    shooter_motor.set(1);
    }

    public void DeactivateShooterMotor()
    {
      shooter_motor.set(0);
      ShooterValue = "OFF";
    }
    
    public void lowShooterSpeed()
    {
      shooter_motor.set(0.5);
      ShooterValue = "LOW SPEED";

    }

    public void highShooterSpeed()
    {
      shooter_motor.set(0.8);
      ShooterValue = "HIGH SPEED";
    }


    // Methods for the Climber
    public void TiltClimber()
    {
      Climber_Solenoid.set(Value.kForward);
      ClimberTiltValue = true;
    }

    public void ReturnClimber()
    {
      Climber_Solenoid.set(Value.kReverse);
      ClimberTiltValue = false;
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
      intake_motor.set(0.5);
      Intake_Solenoid.set(Value.kForward);
      IntakeValue = true;
      intakeTimer = Timer.getFPGATimestamp();
    }

    public void RetractIntake()
    {
      Intake_Solenoid.set(Value.kReverse);
    }

    public void deactivateIntakeMotor()
    {
      intake_motor.set(0);
      IntakeValue = false;
    }

    public void autoPause(double time){
      double pauseStart = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() <= (pauseStart + time) && isAutonomous());
      
    }

    public void autoMove(double time, double speed)
    {
      double autoForwardStart = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() < (autoForwardStart + time) && isAutonomous())
      {
        drive_train.tankDrive(-1 * speed,-1 * speed);
      }
      drive_train.stopMotor();
    }

    public void autoRotate(int degree)
    {
    gyro.zeroYaw();

     while (Math.abs(gyro.getYaw() - degree) > 2 && isAutonomous())
      {
     if (degree > 0)
     {
      drive_train.tankDrive(1, -1);
     }
     if (degree < 0 )
     {
      drive_train.tankDrive(-1, 1);
     }
        
      }
    }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // inverts both sides of the drivetrain(forward on controllers is negative y values)
    left_motor.setInverted(true);
    right_motor.setInverted(true);

    conveyor_motor.setInverted(true);
    shooter_motor.setInverted(true);

    // creates chooser options and displays for primary routines
    m_routines.addOption("Test", test);
    m_routines.addOption("Deposit Cargo and Leave", depositCargoleave);
    m_routines.addOption("Get the Cargo", GetCargo);
    m_routines.addOption("Leave Tarmac", LeaveTarmac);
    m_routines.setDefaultOption("Do nothing", Nothing);
    SmartDashboard.putData("Robot Routines", m_routines);

    // creates chooser options and displays for cargo options
    m_cargochooser.addOption("Wall Cargo", WallCargo);
    m_cargochooser.addOption("Terminal Cargo", TerminalCargo);
    m_cargochooser.addOption("Hangar Cargo", HangarCargo);
    SmartDashboard.putData("Cargo Options", m_cargochooser);

    // creates chooser options and displays for order
    m_order.addOption("Deposit Ball The Fetch and Deposit Other", DepositFirst);
    m_order.addOption("Fetch then Deposit both", FetchFirst);
    SmartDashboard.putData("Order for Get Cargo", m_order);
    
    // creates drive train object of differential drive class
    drive_train = new DifferentialDrive(left_motor, right_motor);

    //makes sure that cargo stopper and climber are in starting position
    ReturnClimber();
    StopCargo();

    // starts camera
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
  public void robotPeriodic() {

    SmartDashboard.putBoolean("Connection Status", gyro.isConnected());
    SmartDashboard.putBoolean("Calibration Status", gyro.isCalibrating());
    SmartDashboard.putNumber("Gyro Angle", gyro.getYaw());
    SmartDashboard.putData("Gyro", gyro);

    SmartDashboard.putBoolean("Climber Tilting", ClimberTiltValue);
    SmartDashboard.putBoolean("Climber Safety", isClimberActivated);
    SmartDashboard.putBoolean("Drive Train Inverted", isDriveTrainInverted);
    SmartDashboard.putBoolean("Conveyor Value", ConveyorValue);
    SmartDashboard.putBoolean("Intake Value", IntakeValue);
    SmartDashboard.putString("Shooter Value", ShooterValue);
    
    SmartDashboard.updateValues();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

   //autonomous vairables
  int autoDepositRotate;
  int autoFetchRotate;



  @Override
  public void autonomousInit() {
    //gets selections from smart dashboard
    m_autoSelected = m_routines.getSelected();
    m_autoCargo = m_cargochooser.getSelected();
    m_autoOrder = m_order.getSelected();
    
    autoTimer.reset();
    autoTimer.start();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);


    switch(m_autoCargo){
      case(WallCargo):{
        autoFetchRotate = 90;
        autoDepositRotate = 90;
      }
      case(TerminalCargo):{
        autoFetchRotate = 90;
        autoDepositRotate = 90;
      }
      case(HangarCargo):{
        autoFetchRotate = 90;
        autoDepositRotate = 90;
      }
    }


    }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // runs specific auto routines depending on selected options
    switch (m_autoSelected) {
      case GetCargo:
      {
        //Routine For if we choose to get cargo
        switch(m_autoOrder){
          //If we choose to fetch the ball first
          case(FetchFirst):{
            ActivateIntake();
            ActivateConveyor();
            autoMove(3, -1);
            deactivateIntakeMotor();
            autoMove(3.5, 1);
            autoRotate(autoFetchRotate);
            lowShooterSpeed();
            autoMove(2,1);
            ReleaseCargo();
            while(isAutonomous());
          } break;

          case(DepositFirst):{
            lowShooterSpeed();
            autoMove(2,1);
            autoRotate(autoDepositRotate);
            ReleaseCargo();
            autoPause(2);
            StopCargo();
            DeactivateShooterMotor();
            autoRotate(-autoDepositRotate);
            ActivateConveyor();
            ActivateIntake();
            autoMove(5, -1);
            while (isAutonomous());


          } break;
        }
      }

      case LeaveTarmac:{
        autoMove(2, .5);
        while(isAutonomous());
        }
      break;
      
      case Nothing:{      
        while(isAutonomous());
        }
      break;

      case depositCargoleave:{
        lowShooterSpeed();
        autoPause(3);
        DeactivateShooterMotor();
        autoMove(4, -1);
        while(isAutonomous());
        }
      break;

      case test: {
        //used for testing functions during autonomous periodic
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
    

    // calls earlier method for updating toggle values
    updateButtonValues();

    // drive train and inverts if inversion is true
    if (isDriveTrainInverted == true)
    {
      drive_train.tankDrive(right_driver_controller.getY() * -1, left_driver_controller.getY() * -1);
    }
    else
    {
      drive_train.tankDrive(left_driver_controller.getY(), right_driver_controller.getY());
    }

    // checks if intake should be on then runs corresponding method
      if(IntakeButton.get()){
      ActivateIntake();
      }
      else{
      RetractIntake();
      }
      if (Timer.getFPGATimestamp() > intakeTimer + 3){
      deactivateIntakeMotor();
      }
      
      // checks if gyro button is being reset then resets gyro if it needs to be reset
      if(GyroResetButton.get())
      {
        gyro.zeroYaw();
      }


      // checks if shooter speed buttons are pressed or off button is pressed then calls corresponding method
      if (ShooterOffButton.get())
      {
        DeactivateShooterMotor();
      }
      else if(HigherShootingSpeedButton.get())
      {
        highShooterSpeed();
      }
      else if(LowerShootingSpeedButton.get())
      {
        lowShooterSpeed();
      }


      // checks if conveyor should be running in a direction or be shut off then runs corresponding method
      if(ConveyorStopButton.get())
      {
        DeactivateConveyor();
      }
      else if(ConveyorForwardButton.get())
      {
        ActivateConveyor();
      }
      else if(ConveyorReverseButton.get())
      {
        ReverseConveyor();
      }


      // checks if climber should be tilted or returned then runs corresponding method
      if(ClimberTiltButton.get()) {
        TiltClimber();
      }
      else if(ClimberReturnButton.get()) {
        ReturnClimber();
      }


      // checks if climber is activated then takes y value as speed motor should be run
      if (isClimberActivated == true)
      {
      climber_extension.set(codriver_controller.getY());
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