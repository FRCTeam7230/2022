/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// software training update
/**
 * 
 * HOW TO DRIVE
 * LEFT JOYSTICK: FORWARD/BACKWARDS
 * RIGHT JOYSTICK: LEFT/RIGHT
 * LEFT TRIGGER(TOGGLE): SHOOT
 * RIGHT TRIGGER(TOGGLE): INTAKE(MANUAL)
 * LEFT BUMPER(TOGGLE): SPEED
 * RIGHT BUMPER(TOGGLE): SLOW
 * B(TOGGLE): AUTOMATIC INTAKE(BROKEN)
 * Y(TAP): INVERT CONTROLS
 * A(TOGGLE): CLIMB
 */
package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
/*Basic Camera Imports*/
//import edu.wpi.first.cameraserver.CameraServer;

/*Advanced Camera Imports*/
import org.opencv.core.Mat;
import org.opencv.core.*;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.RelativeEncoder;
// import java.io.IOException;
// import java.nio.file.Paths;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private String m_autoSelected, colorSelected;
  private String speedStr = "0.3";
  private double shootingPower = 0.3;
  private String climbStr = "0.5";
  private double climbPower = 0.5;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> color_chooser = new SendableChooser<>();
  private Joystick drive_stick = new Joystick(0);
  private Joystick mechanism_stick = new Joystick(1);
  private int width = 160, height = 120;
  private String statusString = "not Running";
  // Pathweaver related
  // private Drivetrain m_Drivetrain = new Drivetrain();
  public DriveSubsystem m_robotDrive = new DriveSubsystem();
  // Speed/turn adjustments in the TankDrive.java file
  private TankDrive m_tTankDrive = new TankDrive(m_robotDrive);
  private boolean tank = false, red = true;
  private static final String arcade = "arcad";
  private static final String tankOption = "tank mod";
  private boolean prevState = false;
  private boolean driveModified = false;  
  public Drivetrain driveTrain = new Drivetrain(m_robotDrive, drive_stick);
  double ballDistance, ballAngleX;
  double initialBallDistance;
  private ThresholdInRange vision = new ThresholdInRange();
  private boolean nowState;

  public static double leftEncoder, rightEncoder;
  /*
  * CAN IDS:
  1 - 4: driving motors
  5: shooter
  6: conveyor
  7: intake
  */
  /*
  Button mapping on Logitech: 
  1: X
  2: A
  3: B
  4: Y
  5: left bumper
  6: right bumper
  7: left trigger
  8: right trigger
  9: back
  10: start
  11: L Joystick Press
  12: R Joystick Press
  */
  private CANSparkMax shooterMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax conveyorMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
  private VictorSPX intakeMotor = new VictorSPX(7);
  private CANSparkMax climberMotor = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);
  private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
  private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Circles");
  NetworkTableEntry bD = table.getEntry("Ball Distance");
  NetworkTableEntry bAngleX = table.getEntry("Ball Angle X");
  NetworkTableEntry bAngleY = table.getEntry("Ball Angle Y");
  private Mechanisms mechanisms = new Mechanisms(mechanism_stick, m_robotDrive, shooterMotor, conveyorMotor, intakeMotor, intakeSolenoid, climberMotor, climberSolenoid);
  private Autonomous auton = new Autonomous(m_robotDrive, shooterMotor, conveyorMotor, intakeMotor, intakeSolenoid, mechanisms);
  private OldAutonomous autonOld = new OldAutonomous(m_robotDrive, shooterMotor, conveyorMotor, intakeMotor, intakeSolenoid);
  // Mechanism (mode id forward backward power)   
  // private Mechanism intake = new Mechanism("button",1,4,7,0.8);
  //  Pathweaver
  static {System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}

   
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */  @Override
  public void robotInit() {
    if (tank){
      m_tTankDrive.initialize();
    }
    m_robotDrive.calibrate();
    
    m_chooser.setDefaultOption("Arcade", arcade);
    m_chooser.addOption("Tank", tankOption);
    SmartDashboard.putData("Driver choices", m_chooser);
    color_chooser.setDefaultOption("Red", "red");
    // color_chooser.setDefaultOption("Blue", "blue");
    color_chooser.addOption("Blue", "blue");
    // color_chooser.addOption("Red", "red");
    SmartDashboard.putData("Color choice", color_chooser);
    
    SmartDashboard.putString("Shooter Adjustment", speedStr);
    SmartDashboard.putString("Climb Adjustment", climbStr);
    SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getVelocity());
    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(width, height);
      camera.setVideoMode(PixelFormat.kYUYV, width, height, 10);
      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Blur", width, height);
      CvSource outputStream2 = CameraServer.putVideo("Target", width, height);
      Mat source = new Mat();
      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        // SmartDashboard.putString("temp", "1");
        //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        //outputStream.putFrame(output);
        String tempColor;
        if (red) {
          tempColor = "red";
        }
        else {
          tempColor = "blue";
        }
          // System.out.println(red);
          Mat processed = vision.process(source, tempColor);
        
        // else {
        //   Mat processed = vision.process(source, "blue");
        // }
        outputStream.putFrame(source);
        outputStream2.putFrame(processed);
      }
    }).start();
}
  /**
   * This functi on is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    ballDistance = vision.getDistance()*0.01;
    ballAngleX = vision.getAngleX();
    m_autoSelected = m_chooser.getSelected();
    colorSelected = color_chooser.getSelected();
    leftEncoder = 96.52 *m_robotDrive.getLeftDistance();
    rightEncoder = -96.52*m_robotDrive.getRightDistance();
    speedStr = SmartDashboard.getString("Shooter Adjustment",Double.toString(shootingPower));
    climbStr = SmartDashboard.getString("Climb Adjustment",Double.toString(climbPower));
    SmartDashboard.putNumber("Turn", m_robotDrive.getTurnRate());
    SmartDashboard.putNumber("LEncoder", leftEncoder);
    SmartDashboard.putNumber("REncoder", rightEncoder); 
    SmartDashboard.putString("Smart Status", statusString);
    SmartDashboard.putNumber("Ball Distance", ballDistance);
    SmartDashboard.putNumber("Angle X", ballAngleX);
    SmartDashboard.putNumber("Shooting Velocity", shooterEncoder.getVelocity());
    bAngleX.setDouble(1);
    bAngleY.setDouble(1);
    bD.setDouble(ballDistance);
    pcmCompressor.enableDigital();
    SmartDashboard.putNumber("Ball Distance", bD.getDouble(ballDistance));  
    switch (m_autoSelected) {
      case tankOption:
        tank = true;
        break;
      case arcade:
        tank = false;
        break;
    }
    switch (colorSelected) {
      case "red":
        red = true;
        break;
      case "blue":
        red = false;
        break;
    }    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    auton.init();
    autonOld.init();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    auton.execute(ballDistance, ballAngleX);
    // autonOld.execute();
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    if (tank){
    m_tTankDrive.execute();
    } 
    // intakeSolenoid.set(true);
    shootingPower = Double.parseDouble(speedStr);
    climbPower = Double.parseDouble(climbStr);
  
    mechanisms.runShotAndIntake(robotConstants.SHOOT_BUTTON,robotConstants.INTAKE_BUTTON, robotConstants.INTAKE_MOTOR_BUTTON, shootingPower, true);
    driveTrain.drive(tank, driveModified);
    mechanisms.runClimber(robotConstants.CLIMBER_UP_BUTTON, robotConstants.CLIMBER_DOWN_BUTTON);
    nowState = drive_stick.getRawButton(robotConstants.SMART_INTAKE_BUTTON);
    if (prevState == false && nowState == true){
      m_robotDrive.resetEncoders();
      initialBallDistance = ballDistance;
    }
    prevState=drive_stick.getRawButton(robotConstants.SMART_INTAKE_BUTTON);
    if (nowState){
        intakeSolenoid.set(nowState);
        intakeMotor.set(ControlMode.PercentOutput, 0.65);
        conveyorMotor.set(0.5);
        // DriverStation.reportWarning("ANGLE: "+Double.toString(angle),true);
        double speed = 0.9;
        double margin = 3;
        double angle = ballAngleX;
        if (angle>0 && angle>margin){
          driveModified = true;
          // DriverStation.reportWarning("angle>0",true);
          m_robotDrive.drive(speed/4, -speed/2);
          statusString = "angle > 0";
        }
        else if (angle<0 && angle<-margin){
          driveModified = true;
          // DriverStation.reportWarning("angle<0",true);
          m_robotDrive.drive(-speed/2, speed/4);
          statusString = "angle < 0";
        }
        else if (Math.abs(angle) < margin){
          driveModified = true;
          // m_robotDrive.resetEncoders();
          // if (leftEncoder<initialBallDistance && rightEncoder < initialBallDistance){
          m_robotDrive.arcadeDrive(0, -speed/2);
          statusString = "angle < margin";
          // }
        }
    }
    else{
      driveModified = false;
    }
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}