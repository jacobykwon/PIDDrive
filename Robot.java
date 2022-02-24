/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private XboxController m_leftStick;
  private XboxController m_rightStick;
  private XboxController m_XButton; //button to make the robot move exactly 10 meters
  private static final int leftDeviceID = 3; 
  private static final int rightDeviceID = 5;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  public final double kP = 0.03;
  public double kI;
  public double kD;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    leftEncoder = new CANEncoder(m_leftMotor);
    rightEncoder = new CANEncoder(m_rightMotor);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);


    m_leftStick = new XboxController(0);
    m_rightStick = new XboxController(1);

  }

  @Override
  public void autonomousInit() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  
  
  double setpoint = 0;

  @Override
  public void autonomousPeriodic() {
    if (m_XButton.getAButton()) {
      setpoint = 10;
    }

    // get sensor position
    double sensorPosition = leftEncoder.getPosition();

    //calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    //output to motors
    m_leftMotor.set(outputSpeed);
    m_rightMotor.set(outputSpeed);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", leftEncoder.getPosition);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getRawAxis(1), m_rightStick.getRawAxis(5));
  }
}
