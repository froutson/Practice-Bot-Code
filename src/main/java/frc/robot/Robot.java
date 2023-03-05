// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import java.util.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DifferentialDrive driveBase;
  private Joystick driverStick1;
  private Joystick driverStick2;
  private Joystick operatorStick;
  private CANSparkMax angleMotor;
  private static final int leftLeaderDeviceID = 10;
  private static final int leftFollowerDeviceID = 14;
  private static final int rightLeaderDeviceID = 12;
  private static final int rightFollowerDeviceID = 13;
  private CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;
  private static final Timer timer = new Timer();
  //pneumatics
  private Compressor phCompressor;
  private DoubleSolenoid gripperSolenoid;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    UsbCamera cam1 = CameraServer.startAutomaticCapture("Drive Camera", 0);
    cam1.setResolution(320, 240);
    cam1.setFPS(10);

    /** 
    UsbCamera cam2 = CameraServer.startAutomaticCapture("Operator Camera", 0);
    cam2.setResolution(160, 120);1`
    cam2.setFPS(15);
    */

    m_chooser.setDefaultOption("No Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftLeader = new CANSparkMax(leftLeaderDeviceID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(leftFollowerDeviceID, MotorType.kBrushless);
    rightLeader = new CANSparkMax(rightLeaderDeviceID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(rightFollowerDeviceID, MotorType.kBrushless);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    driveBase = new DifferentialDrive(leftLeader, rightLeader);

    driverStick1 = new Joystick(0);
    driverStick2 = new Joystick(1);
    operatorStick = new Joystick(2);

    new Timer();

    angleMotor = new CANSparkMax(15, MotorType.kBrushless);

    phCompressor = new Compressor(2, PneumaticsModuleType.REVPH);
    gripperSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 0, 1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.notifyAll();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        
        break;
      case kDefaultAuto:
      default:
        drop();
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    display();
    driveBase.tankDrive(LeftThrottle(), RightThrottle());
    grab();
    angle();
    phCompressor.enableDigital();
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

  public void display() {

  }
  public double RightThrottle() {
    double throttleInput = -driverStick1.getRawAxis(1);
//    return (Math.pow(throttleInput,5))/3  + (Math.pow(throttleInput,3))/3 +throttleInput/3;
      return throttleInput;
  }

  public double LeftThrottle() {
    double throttleInput = driverStick2.getRawAxis(1);
//    return (Math.pow(throttleInput,5))/3  + (Math.pow(throttleInput,3))/3 +throttleInput/3;
      return throttleInput;
  }

public void angle() {
  if (operatorStick.getRawButton(3)) {
    angleMotor.set(.25);
  }
  else if (operatorStick.getRawButton(2)) {
    angleMotor.set(-.25);
  }
  else {
    angleMotor.set(0);
  }
}

public void grab() {
  if (operatorStick.getRawButton(4)) {
    gripperSolenoid.set(Value.kForward);
  }
  else if (operatorStick.getRawButton(5)) {
    gripperSolenoid.set(Value.kReverse);
  }
  
}
public void drop(){
  gripperSolenoid.set(Value.kReverse);
}
}
