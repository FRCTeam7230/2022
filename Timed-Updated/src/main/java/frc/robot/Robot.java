/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;

// import edu.wpi.first.wpilibj.Filesystem;
// import java.nio.file.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.buttons.Button;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.Trajectory.State;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TankDrive;
// import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import java.io.*;
//import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import frc.robot.RobotContainer;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Mechanism;
import com.revrobotics.CANSparkMax;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;

/*Basic Camera Imports*/
//import edu.wpi.first.cameraserver.CameraServer;

/*Advanced Camera Imports*/
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick m_stick = new Joystick(0);
  // Pathweaver related
  // private Drivetrain m_Drivetrain = new Drivetrain();
  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  // Speed/turn adjustments in the TankDrive.java file
  private TankDrive m_tTankDrive = new TankDrive(m_robotDrive);
  private FindPath fp = new FindPath();
  private boolean tank = false;
  private static final String arcade = "arcad";
  private static final String tankOption = "tank mod";
  private CANSparkMax shooterMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
  // Mechanism (mode id forward backward power)
  // private Mechanism intake = new Mechanism("button",1,4,7,0.8);
  //  Pathweaver
   RamseteIterative ramsete = new RamseteIterative(
       fp.getPath(),
       m_robotDrive::getPose,
       new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
       new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                  DriveConstants.kvVoltSecondsPerMeter,
                                  DriveConstants.kaVoltSecondsSquaredPerMeter),
       DriveConstants.kDriveKinematics,
       m_robotDrive::getWheelSpeeds,
       new PIDController(DriveConstants.kPDriveVel, 0, 0),
       new PIDController(DriveConstants.kPDriveVel, 0, 0),
       // RamseteCommand passes volts to the callback
       m_robotDrive::tankDriveVolts
   );
  public void runMechanism(CANSparkMax motor, int button, double power, boolean invert){
    double newPower = power;
    if (invert){
      newPower*=-1;
    }
    if(m_stick.getRawButton(button)){
        DriverStation.reportWarning("running fwd",true);
        motor.set(newPower);
    }
    else{
      motor.set(0);
    }
  } 

 /* public Spark getSpark(int motor)
  {
    switch(motor)
    {
      case 0:
        return l_motor1;
      case 1:
        return r_motor1;
      case 2:
        return l_motor2;
      default:
        return r_motor2;
      
    }
  }*/
 // DifferentialDrive flywheel = new DifferentialDrive(l_flywheel, r_flywheel);

//  RamseteCommand c; 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */  @Override
  public void robotInit() {
    if (tank){
      m_tTankDrive.initialize();
    }
    m_robotDrive.calibrate();
    
    m_chooser.setDefaultOption("Tank", tankOption);
    m_chooser.addOption("Arcade", arcade);
    SmartDashboard.putData("Driver choices", m_chooser);
   //Basic Camera 
    //CameraServer.getInstance().startAutomaticCapture();

    //Advanced Camera
    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();
  }

  

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      SmartDashboard.putNumber("LEncoder", m_robotDrive.getLeftEncoder().getDistance());
      SmartDashboard.putNumber("REncoder", m_robotDrive.getRightEncoder().getDistance());
      SmartDashboard.putNumber("Turn", m_robotDrive.getTurnRate());
      switch (m_autoSelected) {
         case tankOption:
           tank = true;
           break;
         case arcade:
           tank = false;
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
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    // try {
    //Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/YourPath.wpilib.json"));
    // }
    // catch (IOException e)
    // {}
    ramsete.initialize();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    ramsete.execute();
   /* switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
         // Drive for 2 seconds
        if (m_timer.get() < 2.0) {
      //    m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
      } else {
       //   m_robotDrive.stopMotor(); // stop robot
      }
        break;
    }*/
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    if (tank){
    m_tTankDrive.execute();
    }
    // motor, button, power
    runMechanism(shooterMotor, 4, 1.0, true);
    //establishes minimum and maximums of deadzone
    final double deadZone=0.4;
    final double minZone=0.07;
    final double invertAxis = 1;
    final double xOffset = 0.0;
    //gets joystick values and creates curves
    double y = m_stick.getRawAxis(1);
    double yprime = invertAxis * Math.pow(y,3);
      // double yprime=-y;
    double x = m_stick.getRawAxis(2);
    double xprime = Math.pow(x,3);
      // double xprime=x;
    double accelFactor = 2.0;
    double slowFactor = 0.85;
    //The % power used
    final double turnLimit = 0.4;
    double speedLimit=0.5;
    final double leftAdj = 1.3;
    //Reports joystick numbers
    DriverStation.reportWarning("Raw Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
    //Mathmomagic! For X and Y 
    // Y is turn for now
    if(minZone<Math.abs(yprime)){
      // yprime=deadZone*Math.signum(yprime);
        yprime=(Math.abs(yprime)/yprime)*(deadZone+speedLimit*(1-deadZone)*(Math.abs(yprime))/(0.9));
        
        // DriverStation.reportWarning("BANANA"+((Double)Math.abs(yprime)).toString(),true);
    }
    else{
      yprime=0;
    }
    // X is throttle for now
    if(minZone<Math.abs(xprime)){
      xprime=Math.abs(xprime)/xprime*(deadZone+turnLimit*(1-deadZone)*(Math.abs(x)-0.1)/0.9);
      // if (yprime<0){
      //   yprime*=leftAdj;
      // }
        //DriverStation.reportWarning("KIWI"+((Double)Math.abs(xprime)).toString(),true);
    }
    else{
      xprime=0;
    }
    DriverStation.reportWarning("New Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
    // R Bumper is 6
    if(m_stick.getRawButton(5)){
      yprime*=accelFactor;
    }
    
    if(m_stick.getRawButton(6)){
      yprime*=slowFactor;
      xprime*=slowFactor; 
    }
    //Actual drive part
    if (!tank){
      m_robotDrive.arcadeDrive(xprime+xOffset, yprime);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}