// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

// added imports

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;



import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;





/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  
  private static final String kCenter = "Center";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final TalonFX leftLeader = new TalonFX(0);
  private final TalonFX leftFollower = new TalonFX(1);
  private final TalonFX rightLeader = new TalonFX(2);
  private final TalonFX rightFollower = new TalonFX(3);

  private final SparkMax LeftElevator = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightElevator = new SparkMax(5, MotorType.kBrushless);
  private final SparkClosedLoopController elevatorPID = LeftElevator.getClosedLoopController();

  private final SparkMax ArmMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax RollerMotor = new SparkMax(7, MotorType.kBrushless);
  
  private final SparkMax TopEndMotor = new SparkMax(8, MotorType.kBrushless);
  private final SparkMax BottomEndMotor = new SparkMax(9, MotorType.kBrushless);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftLeader::set, rightLeader::set);
  private final Timer m_timer = new Timer();

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();
  private final SparkMaxConfig endConfig = new SparkMaxConfig();
  private final SparkMaxConfig ArmConfig = new SparkMaxConfig();

  private final double ROLLER_EJECT_VALUE = 0.44;
  private double driveSpeed = 1;



  private final XboxController gamepad0Driver = new XboxController(0);
  private final XboxController gamepad1Operator = new XboxController(1);

  



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Center", kCenter);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    /*CameraServer.getServer("CAM")
    CameraServer.getVideo();*/
    configureMotors();
    
    driveConfig.smartCurrentLimit(40);
    driveConfig.voltageCompensation(12);
    driveConfig.softLimit.reverseSoftLimitEnabled(true);
    driveConfig.softLimit.reverseSoftLimit(-119.7);

    driveConfig.closedLoop.p(0.2).i(0.0).d(0.0).outputRange(-1.0, .9);
    driveConfig.closedLoop.velocityFF(1.0/473.0);
    driveConfig.closedLoop.maxMotion.maxVelocity(20.0).maxAcceleration(60.0).allowedClosedLoopError(0.2);

    driveConfig.follow(LeftElevator);
    rightElevator.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveConfig.disableFollowerMode();
    
    LeftElevator.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_robotDrive.setSafetyEnabled(true);
    m_robotDrive.setDeadband(0.02);

    rollerConfig.smartCurrentLimit(40);
    rollerConfig.voltageCompensation(12);
    RollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ArmConfig.smartCurrentLimit(40);
    ArmConfig.voltageCompensation(12);
    ArmMotor.configure(ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    endConfig.smartCurrentLimit(40);
    endConfig.voltageCompensation(12);
    endConfig.follow(TopEndMotor);
    BottomEndMotor.configure(endConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    endConfig.disableFollowerMode();
    TopEndMotor.configure(endConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  } 

  private void configureMotors() {
        // Apply default configurations
        leftLeader.getConfigurator().apply(new TalonFXConfiguration());
        leftFollower.getConfigurator().apply(new TalonFXConfiguration());
        rightLeader.getConfigurator().apply(new TalonFXConfiguration());
        rightFollower.getConfigurator().apply(new TalonFXConfiguration());

        // Set brake mode
        leftLeader.setNeutralMode(NeutralModeValue.Brake);
        leftFollower.setNeutralMode(NeutralModeValue.Brake);
        rightLeader.setNeutralMode(NeutralModeValue.Brake);
        rightFollower.setNeutralMode(NeutralModeValue.Brake);

        // Set motor inversion
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftLeader.getConfigurator().apply(leftConfig);
        leftFollower.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightLeader.getConfigurator().apply(rightConfig);
        rightFollower.getConfigurator().apply(rightConfig);
        

        // Set followers
        leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));



  }     
    
    

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    
  }

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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double driveSpeed = 0.0;
    double endSpeed = 0.0;

    switch (m_autoSelected) {
      case kCenter:
      case kDefaultAuto: 
      default:
        // Put custom auto code here
        if(m_timer.get()< 5){
        driveSpeed = 0.25;
        setPosition = Constants.ELEVATOR_BOTTOM;
      
    }
      else if(m_timer.get()< 8){
        setPosition = Constants.ELEVATOR_L4;


      }
      else if(m_timer.get()< 10){
        setPosition = Constants.ELEVATOR_L4;
        endSpeed = 0.25;

      }
    else {
      setPosition = Constants.ELEVATOR_BOTTOM;
        
    }  
      m_robotDrive.tankDrive(driveSpeed, driveSpeed);
      elevatorPID.setReference(setPosition, ControlType.kPosition);
      TopEndMotor.set(endSpeed);
      BottomEndMotor.set(endSpeed);
      break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }
  
  double setPosition = Constants.ELEVATOR_BOTTOM;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*if(gamepad0Driver.getLeftBumperButton() == true){
      driveSpeed = 2;
  }
    if(gamepad0Driver.getRightBumperButton() == true){
      driveSpeed = 1;
    }*/
    
    /*if(gamepad1Operator.getRightStickButton()){
      //Down
      LeftElevator.set(.4);
      rightElevator.set(.4);
    }
    else if(gamepad1Operator.getLeftStickButton()){
      //Up
      LeftElevator.set(-.4);
      rightElevator.set(-.4);
    }
    else {
      LeftElevator.set(0);
      rightElevator.set(0);
    }*/

    if (gamepad1Operator.getYButtonPressed()){
      setPosition = Constants.ELEVATOR_BOTTOM;
    }
    else if (gamepad1Operator.getBButtonPressed()){
      setPosition = Constants.ELEVATOR_L2;
    }
    else if (gamepad1Operator.getAButtonPressed()){
      setPosition = Constants.ELEVATOR_L3;
    }
    else if (gamepad1Operator.getXButtonPressed()){
      setPosition = Constants.ELEVATOR_L4;
    }

    //Set Elevator Position
    elevatorPID.setReference(setPosition, ControlType.kPosition);

    if (gamepad1Operator.getStartButton())
    {
      DriverStation.reportError("test", true);
      RelativeEncoder elevatorEncoder = LeftElevator.getEncoder();
      elevatorEncoder.setPosition(0.0);
    }

    m_robotDrive.arcadeDrive(-gamepad0Driver.getLeftY()*0.75, -gamepad0Driver.getRightX()*0.75);

    if (gamepad1Operator.getYButtonPressed()){
      setPosition = Constants.ELEVATOR_BOTTOM;
    }
    else if (gamepad1Operator.getBButtonPressed()){
      setPosition = Constants.ELEVATOR_L2;
    }
    else if (gamepad1Operator.getAButtonPressed()){
      setPosition = Constants.ELEVATOR_L3;
    }
    else if (gamepad1Operator.getXButtonPressed()){
      setPosition = Constants.ELEVATOR_L4;
    }

    //Set Elevator Position
    elevatorPID.setReference(setPosition, ControlType.kPosition);

    if (gamepad1Operator.getStartButton())
    {
      DriverStation.reportError("test", true);
      RelativeEncoder elevatorEncoder = LeftElevator.getEncoder();
      elevatorEncoder.setPosition(0.0);
    }

    m_robotDrive.arcadeDrive(-gamepad0Driver.getLeftY()*0.75, -gamepad0Driver.getRightX()*0.75);

    if (gamepad0Driver.getLeftBumperButtonPressed()){
      RollerMotor.set(-0.25);
    }
    if (gamepad0Driver.getLeftBumperButtonReleased()){
      RollerMotor.set(0);
    }
    if (gamepad0Driver.getRightBumperButtonPressed()){
      RollerMotor.set(0.2);
    }
    if (gamepad0Driver.getRightBumperButtonReleased()){
      RollerMotor.set(0);
    }

    
    if (gamepad0Driver.getYButtonPressed()){
      ArmMotor.set(0.2);
    }
    if (gamepad0Driver.getYButtonReleased()){
      ArmMotor.set(0);
    }
    if (gamepad0Driver.getXButtonPressed()){
      ArmMotor.set(-0.2);
    }
    if (gamepad0Driver.getXButtonReleased()){
      ArmMotor.set(0);
    }

    

    
    if (gamepad1Operator.getLeftBumperButton()){
      //out
      TopEndMotor.set(.25);
      BottomEndMotor.set(.25);
    }
    else if (gamepad1Operator.getRightBumperButton()){
      //in
      TopEndMotor.set(-.25);
      BottomEndMotor.set(-.25);
    }
    else {
      TopEndMotor.set(.0);
      BottomEndMotor.set(.0);
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
