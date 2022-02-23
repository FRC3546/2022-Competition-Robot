
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Timer has been changed to autoTimer and reset/start
//has been moved to autonomousinit to fix timing problems while testing autonomous

/**
 * CHANGE LOG:
 * 2/17/22 CF: Changed timer name to auto timer & created autoPause() 
 * 2/18/22 BAC: Added Change Log
 * 2/18/22 JMF: Fixed autonomous by adding missing functions and changed gyro directions
 * 2/19/22 JMF: Changed solenoid IO location, fixed autonomous direction for leaveTarmac and solenoid values, 
 * renamed every variable to be more uniformed, and adjusted conveyor speed.
 * 2/19/22 CF: Modified autonomous values and added heading maitnence system.
 * 2/20/22 BAC: New conveyer logic, associated with intake,shooting or button press
 * 2/20/22 BAC: New low/high cargo release buttons to automate shooter motor; cleaned up indenting/spacing
 * 2/20/22 CF: Added commands to teleop init so that everything is set back to defaults/off
 * 2/21/22 CF: Swapped Double Solenoids for climber tilt
 * 2/21/22 CF: Changed intake motor deactivation delay to one and a half second
 * 2/21/22 CF: changed turn values for auto routine to .75 speed
 * 
 * 
 * 
 * 
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
  private DifferentialDrive driveTrain;
  
  // These are the definitions of the actual joysticks
  private Joystick leftDriverController= new Joystick(0);
  private Joystick rightDriverController = new Joystick(1);
  private Joystick coDriverController = new Joystick(2);
  
  // Driver right controller buttons
  private JoystickButton intakeButton = new JoystickButton(rightDriverController, 1);
  private JoystickButton driveTrainReturnButton = new JoystickButton(rightDriverController, 6);
  private JoystickButton driveTrainInvertButton = new JoystickButton(rightDriverController, 7);
  
  // Driver left controller buttons
  private JoystickButton gyroResetButton = new JoystickButton(leftDriverController, 10);

  // Co-driver buttons
  //private JoystickButton higherShootingSpeedButton = new JoystickButton(coDriverController, 7);
  //private JoystickButton shooterOffButton = new JoystickButton(coDriverController, 9);
  //private JoystickButton lowerShootingSpeedButton = new JoystickButton(coDriverController, 11);
  private JoystickButton conveyorForwardButton = new JoystickButton(coDriverController, 8);
  // private JoystickButton conveyorStopButton = new JoystickButton(coDriverController, 10);
  private JoystickButton conveyorReverseButton = new JoystickButton(coDriverController, 12);
  private JoystickButton climberTiltButton = new JoystickButton(coDriverController, 5);
  private JoystickButton climberReturnButton = new JoystickButton(coDriverController, 3);
  private JoystickButton climberActivationButton = new JoystickButton(coDriverController, 6);
  private JoystickButton climberDeactivationButton = new JoystickButton(coDriverController, 4);
  private JoystickButton lowCargoReleaseButton = new JoystickButton(coDriverController, 1);
  private JoystickButton highCargoReleaseButton = new JoystickButton(coDriverController, 2);


  // Creates double solenoids for future reference
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid cargoReleaseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  private DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);

  // values for toggle(with seperate buttons) buttons
  private boolean isDriveTrainInverted = false;
  private boolean isClimberActivated = false;
  private boolean intakeValue = false;
  private boolean conveyorValue = false;
  private boolean climberTiltValue = false;
  private String shooterValue = "OFF";

  // creates game timer
  private final Timer autoTimer = new Timer();

  private double intakeTimer;


  // values for which auto routine we are using used when we pull which selector we have choosen
  private String autoSelected;
  private String autoCargo;
  private String autoOrder;

  // chooser for primary routine(defaults as doing nothing)
  private static final String test = "test";
  private static final String depositCargoleave = "Deposit Cargo then Leave";
  private static final String getCargo = "Get the Cargo";
  private static final String leaveTarmac = "Leave Tarmac";
  private static final String Nothing = "Do Nothing";
  private final SendableChooser<String> routines = new SendableChooser<>();
  
  // chooser get cargo(determines which cargo we are going for from set positions)
  private static final String wallCargo = "Wall Cargo";
  private static final String terminalCargo = "Terminal Cargo";
  private static final String hangarCargo = "Hanger Cargo";
  private final SendableChooser<String> cargoChooser = new SendableChooser<>();

  // chooser for order(if we get cargo)
  private static final String depositFirst = "Deposit First";
  private static final String fetchFirst = "Fetch Ball First";
  private final SendableChooser<String> order = new SendableChooser<>();

  // creates drive train victorSP motor controllers
  private VictorSP leftMotor = new VictorSP(0);
  private VictorSP rightMotor = new VictorSP(2);

  // creates other motor controllers
  private VictorSP intakeMotor = new VictorSP(4);
  private VictorSP conveyorMotor = new VictorSP(6);
  private Spark climberExtension = new Spark(7);
  private CANSparkMax shooterMotor = new CANSparkMax(28, MotorType.kBrushless);

  // creates gyro object for navx board
  AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  final double kP = 1;

  // method for finding our toggle button values
  public void updateButtonValues() {
    if (driveTrainInvertButton.get()) {
      isDriveTrainInverted = true;
    }
    else if (driveTrainReturnButton.get()) {
      isDriveTrainInverted = false;
    }
    if (climberActivationButton.get()) {
      isClimberActivated = true;
    }
    else if (climberDeactivationButton.get()) {
      isClimberActivated = false;
    }
  }

  // Methods for the Conveyor
  public void ActivateConveyor() {
    conveyorMotor.set(0.8);
    conveyorValue = true;
  }

  public void DeactivateConveyor() {
    conveyorMotor.set(0);
    conveyorValue = false;
  }

  public void ReverseConveyor() {
    conveyorMotor.set(-0.5);
    conveyorValue = true;
  } 


  public void DeactivateShooterMotor() {
    shooterMotor.set(0);
    shooterValue = "OFF";
  }
    
  public void lowShooterSpeed() {
    shooterMotor.set(0.7);
    shooterValue = "LOW SPEED";
  }

  public void highShooterSpeed() {
    shooterMotor.set(0.8);
    shooterValue = "HIGH SPEED";
  }

  // Methods for the Climber
  public void TiltClimber() {
    climberSolenoid.set(Value.kReverse);
    climberTiltValue = true;
  }

  public void ReturnClimber() {
    climberSolenoid.set(Value.kForward);
    climberTiltValue = false;
  }

  // Methods for the Cargo
  public void ReleaseCargo() {
    cargoReleaseSolenoid.set(Value.kReverse);
  }

  public void StopCargo() {
    cargoReleaseSolenoid.set(Value.kForward);
  }

  // Methods for Intake
  public void ActivateIntake() {
    intakeMotor.set(0.5);
    intakeSolenoid.set(Value.kForward);
    intakeValue = true;
    intakeTimer = Timer.getFPGATimestamp();
  }

  public void RetractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void deactivateIntakeMotor() {
    intakeMotor.set(0);
    intakeValue = false;
  }

  public void autoPause(double time) {
    double pauseStart = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() <= (pauseStart + time) && isAutonomous());
  }

  public void autoMove(double time, double speed) {
    // double autoHeading = gyro.getAngle();
    // double error;
    double autoForwardStart = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() < (autoForwardStart + time) && isAutonomous()) {
      // error = autoHeading - gyro.getAngle();
      driveTrain.tankDrive(-1 * speed,-1 * speed);
    }
    driveTrain.stopMotor();
  }

  public void autoRotate(int degree) {
    gyro.zeroYaw();

    while (Math.abs(gyro.getAngle() - degree) > 2 && isAutonomous()) {
      System.out.println(gyro.getAngle());
      if (degree > 0) {
        System.out.println("Right" + degree);
        driveTrain.tankDrive(-.75, .75);
        
      }
      if (degree < 0 ) {
        System.out.println("Left" + degree);
        driveTrain.tankDrive(.75, -.75);
      }
    }
    driveTrain.stopMotor();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // inverts both sides of the drivetrain(forward on controllers is negative y values)
    leftMotor.setInverted(true);
    rightMotor.setInverted(true);

    conveyorMotor.setInverted(true);
    shooterMotor.setInverted(true);

    // creates chooser options and displays for primary routines
    routines.addOption("Test", test);
    routines.addOption("Deposit Cargo and Leave", depositCargoleave);
    routines.addOption("Get the Cargo", getCargo);
    routines.addOption("Leave Tarmac", leaveTarmac);
    routines.setDefaultOption("Do nothing", Nothing);
    SmartDashboard.putData("Robot Routines", routines);

    // creates chooser options and displays for cargo options
    cargoChooser.addOption("Wall Cargo", wallCargo);
    cargoChooser.addOption("Terminal Cargo", terminalCargo);
    cargoChooser.addOption("Hangar Cargo", hangarCargo);
    cargoChooser.setDefaultOption("Do nothing", Nothing);
    SmartDashboard.putData("Cargo Options", cargoChooser);

    // creates chooser options and displays for order
    order.addOption("Deposit Ball The Fetch and Deposit Other", depositFirst);
    order.addOption("Fetch then Deposit both", fetchFirst);
    order.setDefaultOption("Do nothing", Nothing);
    SmartDashboard.putData("Order for Get Cargo", order);
    
    // creates drive train object of differential drive class
    driveTrain = new DifferentialDrive(leftMotor, rightMotor);

    //makes sure that cargo stopper and climber are in starting position
    ReturnClimber();
    StopCargo();
    RetractIntake();

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

    //puts values from code onto smart dashboard
    SmartDashboard.putBoolean("Connection Status", gyro.isConnected());
    SmartDashboard.putBoolean("Calibration Status", gyro.isCalibrating());
    SmartDashboard.putNumber("Gyro Angle", gyro.getYaw());
    SmartDashboard.putData("Gyro", gyro);

    SmartDashboard.putBoolean("Climber Tilting", climberTiltValue);
    SmartDashboard.putBoolean("Climber Safety", isClimberActivated);
    SmartDashboard.putBoolean("Drive Train Inverted", isDriveTrainInverted);
    SmartDashboard.putBoolean("Conveyor Value", conveyorValue);
    SmartDashboard.putBoolean("Intake Value", intakeValue);
    SmartDashboard.putString("Shooter Value", shooterValue);
    //updates vales for smart dashboard
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
    autoSelected = routines.getSelected();
    autoCargo = cargoChooser.getSelected();
    autoOrder = order.getSelected();
    
    autoTimer.reset();
    autoTimer.start();
    // autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + autoSelected);


    //These are the rotation angles used for shooting into the lower hub based on whichever ball we are collecting.
    switch(autoCargo) {
      case(wallCargo): {
        autoFetchRotate = 0;
        autoDepositRotate = 0;
      }
      case(terminalCargo): {
        autoFetchRotate = -35;
        autoDepositRotate = -35;
      }
      case(hangarCargo): {
        autoFetchRotate = -15;
        autoDepositRotate = -15;
      }
      case(Nothing): {
        // does nothing
      }
    }


    }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // runs specific auto routines depending on selected options
    switch (autoSelected) {
      case getCargo: {

        //Routine For if we choose to get cargo
        switch(autoOrder) {
          case(fetchFirst): { //If we choose to fetch the cargo first
            ActivateIntake();
            ActivateConveyor();
            autoMove(2.5, -.5);
            // deactivateIntakeMotor();
            RetractIntake();
            autoMove(4, .5);
            autoRotate(autoFetchRotate);
            lowShooterSpeed();
            ReleaseCargo();
            while(isAutonomous());
          }
          break;

          case(depositFirst): { //If we choose to deposit the cargo first
            lowShooterSpeed();
            ActivateConveyor();
            autoMove(1,.5);
            autoRotate(autoDepositRotate);
            ReleaseCargo();
            autoPause(2);
            StopCargo();
            DeactivateShooterMotor();
            autoRotate(-autoDepositRotate);
            ActivateIntake();
            autoMove(3.5, -.5);
            while (isAutonomous());
          }
          break;

          case(Nothing): {
            // does nothing
          }
        }
      }
      case leaveTarmac: { //If we choose to simply leave the Tarmac
        autoMove(2, -0.5);
        while(isAutonomous());
        }
      break;
      case Nothing: {      
        while(isAutonomous());
        }
        break;
      case depositCargoleave: { //If we choose to deposit the cargo and then leave the Tarmac
        lowShooterSpeed();
        ActivateConveyor();
        autoPause(2);
        ReleaseCargo();
        autoPause(2);
        DeactivateShooterMotor();
        autoMove(3, -.7);
        while(isAutonomous());
        }
        break;
      case test: { //used for testing functions during autonomous periodic
        }
        break;
      }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
//These commands set everything off/default for when control is transfered to human players
DeactivateConveyor();
DeactivateShooterMotor();
StopCargo();
ReturnClimber();
isClimberActivated = false;
isDriveTrainInverted = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    

    // calls earlier method for updating toggle values
    updateButtonValues();

    // drive train and inverts if inversion is true
    if (isDriveTrainInverted == true) {
      driveTrain.tankDrive(rightDriverController.getY() * -1, leftDriverController.getY() * -1);
    }
    else {
      driveTrain.tankDrive(leftDriverController.getY(), rightDriverController.getY());
    }

    // checks if intake should be on then runs corresponding method
    if (intakeButton.get()) {
      ActivateIntake();
    }
    else {
      RetractIntake();
    }
    if (Timer.getFPGATimestamp() > intakeTimer + 1.5){
      deactivateIntakeMotor();
    }
      
    // checks if gyro button is being reset then resets gyro if it needs to be reset
    if (gyroResetButton.get()) {
      gyro.zeroYaw();
    }

    /* // checks if shooter speed buttons are pressed or off button is pressed then calls corresponding method
    if (shooterOffButton.get()) {
      DeactivateShooterMotor();
    }
    else if (higherShootingSpeedButton.get()) {
      highShooterSpeed();
    }
    else if (lowerShootingSpeedButton.get()) {
      lowShooterSpeed();
    } */


    // checks if conveyer should be running in a direction or be shut off then runs corresponding method
    if (conveyorForwardButton.get() || intakeValue || (!shooterValue.equals("OFF"))) {
      ActivateConveyor();
    }
    else if (conveyorReverseButton.get()) {
      ReverseConveyor();
    }
    else {
      DeactivateConveyor();
    }


    /* // checks if conveyor should be running in a direction or be shut off then runs corresponding method
    if (conveyorStopButton.get()) {
      DeactivateConveyor();
    }
    else if (conveyorForwardButton.get()) {
      ActivateConveyor();
    }
    else if (conveyorReverseButton.get()) {
      ReverseConveyor();
    }
 */

    // checks if climber should be tilted or returned then runs corresponding method
    if (climberTiltButton.get()) {
      TiltClimber();
    }
    else if (climberReturnButton.get()) {
      ReturnClimber();
    }


    // checks if cargo should be released at low or high speed and runs corresponding methods
    if (lowCargoReleaseButton.get()) {
      lowShooterSpeed();
      ReleaseCargo();
    }
    else if (highCargoReleaseButton.get()) {
      highShooterSpeed();
      ReleaseCargo();
    }
    else {
      StopCargo();
      DeactivateShooterMotor();
    }


    // checks if climber is activated then takes y value as speed motor should be run
    if (isClimberActivated == true) {
      climberExtension.set(coDriverController.getY());
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