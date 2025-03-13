// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



// added imports

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;


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

  private final SparkMax RollerMotor = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax ArmMotor = new SparkMax(7, MotorType.kBrushless);
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

    configureMotors();

    driveConfig.smartCurrentLimit(60);
    driveConfig.voltageCompensation(12);

    //driveConfig.follow(LeftElevator);
    rightElevator.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //driveConfig.disableFollowerMode();
    
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
    BottomEndMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    endConfig.disableFollowerMode();
    TopEndMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  } 

  private void configureMotors() {
        // Apply default configurations
        leftLeader.getConfigurator().apply(new MotorOutputConfigs());
        leftFollower.getConfigurator().apply(new MotorOutputConfigs());
        rightLeader.getConfigurator().apply(new MotorOutputConfigs());
        rightFollower.getConfigurator().apply(new MotorOutputConfigs());

        // Set brake mode
        leftLeader.setNeutralMode(NeutralModeValue.Brake);
        leftFollower.setNeutralMode(NeutralModeValue.Brake);
        rightLeader.setNeutralMode(NeutralModeValue.Brake);
        rightFollower.setNeutralMode(NeutralModeValue.Brake);

        // Set motor inversion
        MotorOutputConfigs leftConfig = new MotorOutputConfigs();
        leftConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        leftLeader.getConfigurator().apply(leftConfig);
        leftFollower.getConfigurator().apply(leftConfig);

        MotorOutputConfigs rightConfig = new MotorOutputConfigs();
        rightConfig.Inverted = InvertedValue.Clockwise_Positive;
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
    switch (m_autoSelected) {
      case kCenter:
        // Put custom auto code here
        if(m_timer.get()< 3){
          m_robotDrive.tankDrive(0.5, 0.5);
        }
        if (m_timer.get()< 5){
          m_robotDrive.tankDrive(0.5, -0.5);
        } else {
          m_robotDrive.tankDrive(0, 0);
      }  
      break;
      case kDefaultAuto: 
      default:
        // Put default auto code here
        m_robotDrive.tankDrive(0.5, 0.5);
      break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*if(gamepad0Driver.getLeftBumperButton() == true){
      driveSpeed = 2;
  }
    if(gamepad0Driver.getRightBumperButton() == true){
      driveSpeed = 1;
    }*/
    
    if(gamepad1Operator.getRightStickButtonPressed()){
      
      LeftElevator.set(.4);
      if (gamepad1Operator.getRightStickButtonReleased()){
        LeftElevator.set(0);
      }
    }

    if(gamepad1Operator.getLeftStickButtonPressed() == true){
      
      LeftElevator.set(-.4);
      if (gamepad1Operator.getLeftStickButtonReleased()){
        LeftElevator.set(0);
      }
    }
    m_robotDrive.arcadeDrive(-gamepad0Driver.getLeftY(), -gamepad0Driver.getRightX());

    if (gamepad1Operator.getBButtonPressed()){
      RollerMotor.set(0.5);
    }
    if (gamepad1Operator.getBButtonReleased()){
      RollerMotor.set(0);
    }
    if (gamepad1Operator.getAButtonPressed()){
      RollerMotor.set(-0.5);
    }
    if (gamepad1Operator.getAButtonReleased()){
      RollerMotor.set(0);
    }
    if (gamepad1Operator.getXButtonPressed()){
      ArmMotor.set(0.4);
    }
    if (gamepad1Operator.getXButtonReleased()){
      ArmMotor.set(0);
    }
    if (gamepad1Operator.getYButtonPressed()){
      ArmMotor.set(-0.4);
    }
    if (gamepad1Operator.getYButtonReleased()){
      ArmMotor.set(0);
    }
    if (gamepad1Operator.getLeftBumperButtonPressed()){
      TopEndMotor.set(.5);
      BottomEndMotor.set(.5);
      if (gamepad1Operator.getLeftBumperButtonReleased()){
        TopEndMotor.set(.0);
        BottomEndMotor.set(.0);
      }
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
