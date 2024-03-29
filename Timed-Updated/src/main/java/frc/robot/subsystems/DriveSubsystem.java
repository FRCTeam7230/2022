/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.robotConstants;
import com.kauailabs.navx.frc.AHRS; 
//import frc.robot.Robot;
//import frc.robot.Robot;
public class DriveSubsystem extends SubsystemBase {
  public double initialLeft, initialRight;
  //test
  
    // public Spark l_motor1=  new Spark(0);
    // public Spark r_motor1 = new Spark(2);
    // public Spark l_motor2 = new Spark(1);
    // public Spark r_motor2 = new Spark(3);
  public CANSparkMax l_motor1 = new CANSparkMax(robotConstants.L1MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax r_motor1 = new CANSparkMax(robotConstants.R1MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax l_motor2 = new CANSparkMax(robotConstants.L2MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  public CANSparkMax r_motor2 = new CANSparkMax(robotConstants.R2MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  // public SparkMaxPIDController lController1 = l_motor1.getPIDController();
  // public SparkMaxPIDController rController1 = r_motor1.getPIDController();
  // public SparkMaxPIDController lController2 = l_motor2.getPIDController();
  // public SparkMaxPIDController rController2 = r_motor2.getPIDController();
  // The motors on the left side of the drive.
  // private final SpeedControllerGroup m_leftMotors =
  // new SpeedControllerGroup(l_motor1,
  //                          l_motor2);
  private int maxAccel;
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(l_motor1,
                               l_motor2);

  // The motors on the right side of the drive.
  // private final SpeedControllerGroup m_rightMotors =
  //     new SpeedControllerGroup(r_motor1,
  //                              r_motor2);

  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(r_motor1,
                               r_motor2);
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = 
  l_motor1.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder = 
  r_motor1.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // The gyro sensoroo
  private final Gyro m_gyro = new AHRS();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setPositionConversionFactor​(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setPositionConversionFactor​(DriveConstants.kEncoderDistancePerPulse);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setInverted(true);
    // r_motor1.setInverted(true);
    initialLeft = m_leftEncoder.getPosition();
    initialRight = m_rightEncoder.getPosition();
    // lController1.setSmartMotionMaxAccel(maxAccel, 0);
    // rController1.setSmartMotionMaxAccel(maxAccel, 0);
    // lController2.setSmartMotionMaxAccel(maxAccel, 0);
    // rController2.setSmartMotionMaxAccel(maxAccel, 0);
    // l_motor1.setSmartCurrentLimit(20);
    // l_motor2.setSmartCurrentLimit(20);
    // r_motor1.setSmartCurrentLimit(20);
    // r_motor2.setSmartCurrentLimit(20);
    m_drive.setSafetyEnabled(false);
    // resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
                      m_rightEncoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  public void calibrate(){
    m_gyro.reset();
  }
  

  public void drive(double left, double right) {
    m_drive.tankDrive(left, -right);
}
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    initialLeft = m_leftEncoder.getPosition();
    initialRight = m_rightEncoder.getPosition();
    
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public double getLeftDistance(){
    return m_leftEncoder.getPosition()-initialLeft;
  }
  public double getRightDistance(){
    return m_rightEncoder.getPosition()-initialRight;
  }
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  public double getError(){
    return m_leftEncoder.getPosition()-m_rightEncoder.getPosition();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
}