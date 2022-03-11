/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Filesystem;
// import java.nio.file.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
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

import java.lang.Math;
import java.util.Collections;
import java.util.List;
import java.util.SortedSet;

import javax.naming.directory.ModificationItem;

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
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
/*Basic Camera Imports*/
//import edu.wpi.first.cameraserver.CameraServer;

/*Advanced Camera Imports*/
import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private String m_autoSelected, colorSelected;
  private String speedStr = "0.6";
  private double shootingPower = 0.6;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> color_chooser = new SendableChooser<>();
  private Joystick m_stick = new Joystick(0);
  private Timer m_timer = new Timer();
  private Timer c_timer = new Timer();
  // Pathweaver related
  // private Drivetrain m_Drivetrain = new Drivetrain();
  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  // Speed/turn adjustments in the TankDrive.java file
  private TankDrive m_tTankDrive = new TankDrive(m_robotDrive);
  private FindPath pathOne = new FindPath("/home/lvuser/deploy/paths/Path1.wpilib.json");
  private FindPath pathTwo = new FindPath("/home/lvuser/deploy/paths/Path2.wpilib.json");
  private boolean tank = false, red = true;
  private static final String arcade = "arcad";
  private static final String tankOption = "tank mod";
  private boolean prevState = false;
  private boolean prevState2 = false;
  private boolean driveModified = false;  
  private boolean firstActivated = true, secondActivated = true;
  private boolean doneFirst=false, doneSecond=false, doneThird = false;
  // private ThresholdInRange vision = new ThresholdInRange();
  //private DigitalInput initialConveyerSensor;
  //private DigitalInput finalConveyerSensor; 
  private DigitalInput IRSensor1;
  private DigitalInput IRSensor2;
  private boolean resetBool, nowState;
  private double leftEncoder, rightEncoder;
  private String autoState;
  /*
  * CAN IDS:
  1 - 4: driving motors
  5: shooter
  6: conveyor
  */
  // TODO: Change the ID of shooterMotor, or use different motor controllers
  private CANSparkMax shooterMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax conveyorMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
  private VictorSPX intakeMotor = new VictorSPX(7);
  private Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Circles");
  NetworkTableEntry bD = table.getEntry("Ball Distance");
  NetworkTableEntry bAngleX = table.getEntry("Ball Angle X");
  NetworkTableEntry bAngleY = table.getEntry("Ball Angle Y");
  // Mechanism (mode id forward backward power)   
  // private Mechanism intake = new Mechanism("button",1,4,7,0.8);
  //  Pathweaver
  static {System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}

   RamseteIterative ramsete1 = new RamseteIterative(
       pathOne.getPath(),
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
   RamseteIterative ramsete2 = new RamseteIterative(
       pathTwo.getPath(),
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

   public void runCANMechanism(CANSparkMax motor, int button, double power, boolean invert, double offPower){
    // boolean previousState = false;
    boolean state = m_stick.getRawButton(button); 
    double newPower = power;
     if (invert){
       newPower*=-1;
     }
    //  if (state && !previousState){
    //    motor.set(-offPower);
    //  }
    //  previousState = state;
     if(state){
        //  DriverStation.reportWarning("running button "+button+"fwd",true);
         motor.set(newPower);
     }
     else{
       motor.set(-offPower);
     }
     
   } 

   public void runSPXMechanism(VictorSPX motor, int button, double power, boolean invert){
    double newPower = power;
    if (invert){
      newPower*=-1;
    }
    if(m_stick.getRawButton(button)){
        // DriverStation.reportWarning("running spxbutton "+button+"fwd",true);
        motor.set(ControlMode.PercentOutput,newPower);
    }
    else{
      motor.set(ControlMode.PercentOutput,0);
    }
  } 

   public void runPneumaticCompressor(Compressor comp, int button, boolean enabled){
    if(m_stick.getRawButton(button)){
        // DriverStation.reportWarning("running compressor",true);
        comp.enableDigital();

    }
    else{
      // DriverStation.reportWarning("compressor off",true);
      comp.disable();
    }
  } 
  public void runPneumaticSolenoid(Solenoid solenoid, int button, boolean enabled){
   if(m_stick.getRawButton(button) && enabled){
        // DriverStation.reportWarning("running solenoid",true);
        solenoid.set(true);

   }
   else if (enabled){
        // DriverStation.reportWarning("solenoid off",true);
        solenoid.set(false);
     
   }
 } 
  public void runClimber(int button, double speed, CANSparkMax motor, Solenoid solenoid){
    c_timer.reset();
    c_timer.start();
    if (m_timer.get() < 4.0){
      solenoid.set(true);
      if (m_timer.get()>1.0){
        motor.set(speed);
      }
    }
    if (m_timer.get() > 4.0 && m_timer.get() < 8.0){
      solenoid.set(false);
      if (m_timer.get()>5.0 && m_timer.get() < 8.0){
        motor.set(speed);
      }
    }
    
  }
// button1 = shoot, button2 = intake
  public void runShotAndIntake(int button1, int button2, double power, boolean enabled){
    boolean state1 = m_stick.getRawButton(button1);
    boolean state2 = m_stick.getRawButton(button2); 
      double newPower = power;
      //  if (state && !previousState){
      //    motor.set(-offPower);
      //  }
      //  previousState = state;
      if(state1){
        // DriverStation.reportWarning("running button "+button1+"fwd",true);
        shooterMotor.set(newPower);
        conveyorMotor.set(0.5);
      }
      else{
        shooterMotor.set(-newPower/10);
      }
      if(state2){
          // DriverStation.reportWarning("running button "+button2+"fwd",true);
          conveyorMotor.set(0.5);
      }
      if (!state1 && !state2){
        conveyorMotor.set(0);
      }
      runPneumaticSolenoid(intakeSolenoid, button2, !enabled);
      runSPXMechanism(intakeMotor, button2, 0.5, false);
  }

  public boolean driveSetDistance(double distance, double speed){
    boolean finished = false;
    if (Math.abs(leftEncoder)<Math.abs(distance) && Math.abs(rightEncoder) < Math.abs(distance)){
      m_robotDrive.arcadeDrive(0, speed);
      // DriverStation.reportWarning(Double.toString(leftEncoder) + ", "+ Double.toString(distance), true);
      
    }
    else {
      finished = true;
    }
    return finished;
  }
// intakeMotor.set(ControlMode.PercentOutput, 0.4)
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
    
    m_chooser.setDefaultOption("Arcade", arcade);
    m_chooser.addOption("Tank", tankOption);
    SmartDashboard.putData("Driver choices", m_chooser);
    color_chooser.setDefaultOption("Red", "red");
    color_chooser.addOption("Blue", "blue");
    SmartDashboard.putData("Color choice", color_chooser);
    
    // speedStr = SmartDashboard.getString("Shooter Speed","0.7");
    SmartDashboard.putString("Shooter Speed", speedStr);
   //Basic Camera 
    //CameraServer.getInstance().startAutomaticCapture();

    //initialConveyorSensor = new DigitalInput(4);
    //finalConveyerSensor = new DigitalInput(5);  
    IRSensor1 = new DigitalInput(7);
    IRSensor2 = new DigitalInput(8);

    //Advanced Camera
    Thread m_visionThread;
    
    if (tank){
      m_tTankDrive.initialize();
    }
    m_robotDrive.calibrate();
    m_robotDrive.resetEncoders();
    m_chooser.setDefaultOption("Arcade", arcade);
    m_chooser.addOption("Tank", tankOption);
    SmartDashboard.putData("Driver choices", m_chooser);
    
    // speedStr = SmartDashboard.getString("Shooter Speed","0.7");
    SmartDashboard.putString("Shooter Speed", speedStr);
   //Basic Camera 
    //CameraServer.getInstance().startAutomaticCapture();

    //initialConveyorSensor = new DigitalInput(4);
    //finalConveyerSensor = new DigitalInput(5);
    // IRSensor1 = new DigitalInput(7);
    // IRSensor2 = new DigitalInput(8);

    //Advanced Camera
    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setVideoMode(PixelFormat.kYUYV, 320, 240, 10);

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);
      CvSource outputStream2 = CameraServer.putVideo("Target", 320, 240);

      //Mat source = new Mat();
      //Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        //outputStream.putFrame(output);
        Mat processed = process(source);
        outputStream.putFrame(source);
        outputStream2.putFrame(processed);
        
      }
    }).start();
  }


  public static int varForTimer = 0;
  private static int screenCenterX = 160;
  private static int screenCenterY = 120;
  
  //screen size: x= 634, y =  480
  
  //measurement:
  private static double focalLength = 320.8; //focal length in pixels
  private static double ballRadius = 12.5;
  private static double distanceCameraToBall = 0;
  
  private static double depth = ballRadius;
  private static int robotDepth = 13;
  private static double cameraAngle = 55.0;//change this to another angle from flour
  public static double ballDistance;
  public static double ballAngleX;
  public static double ballAngleY;

  private static Mat thresh = new Mat();
  private static Mat frameHSV = new Mat();
  private static Mat circles = new Mat();
  private static Mat source = new Mat();
  
private Mat process(Mat frame) {
  
  // SmartDashboard.putString("rr: ", "11");
  // Mat frame = frames.get(frames.size() - 1);
  //Mat frameHSV = new Mat();  
  Imgproc.cvtColor(frame, frameHSV, Imgproc.COLOR_BGR2HSV);
  //Mat thresh = new Mat();
  
  //red color - need to change
  if (red){
  Core.inRange(frameHSV, new Scalar(0, 130, 130),
         new Scalar(20, 240, 255), thresh);
  }
  else{
  Core.inRange(frameHSV, new Scalar(95, 50, 0),
          new Scalar(110, 255, 255), thresh);
  }

List<Mat> frames = new ArrayList<Mat>();//new List<Mat>();
  Core.split(thresh, frames);
  Mat gray = frames.get(0);
  // System.out.println("Test");
//Default:
//            Core.inRange(frameHSV, new Scalar(sliderLowH.getValue(), sliderLowS.getValue0(), sliderLowV.getValue()),
//                    new Scalar(sliderHighH.getValue(), sliderHighS.getValue(), sliderHighV.getValue()), thresh);

  // Imgproc.putText(frame, ".", new Point(screenCenterX, screenCenterY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 3);	
  Imgproc.medianBlur(gray, gray, 5);
  //Mat circles = new Mat();    	        	  
    
  Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 2.0,
            2*(double)gray.rows(), // change this value to detect circles with different distances to each other
            50.0, 30.0, 0, 0); // change the last two parameters
                  // (min_radius & max_radius) to detect larger circles - need to change min radius to normal values
  for (int x = 0; x < circles.cols(); x++) {
    // SmartDashboard.putString("TestA: ", "0");
    // System.out.println("TestA");

    double[] c = circles.get(0, x);
              
      int cX = (int) Math.round(c[0]/5 - 1)*5; //coordinatesX and coordinatesY
      int cY = (int) Math.round(c[1]/5 - 1)*5;
      Point center = new Point(cX, cY);
      int radius = (int) Math.round(c[2]);
              
      String coordinateXY = cX + "," + cY;
              
              // circle center
      // Imgproc.circle(frame, center, 1, new Scalar(0,255,100), 3, 8, 0);
              // circle outline
      Imgproc.circle(frame, center, radius, new Scalar(255,0,255), 3, 8, 0);
              
      // Imgproc.putText(frame, coordinateXY, new Point(cX, cY), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 111), 2);
            //distance from camera to ball
          distanceCameraToBall = Math.round(focalLength*ballRadius/radius) - depth;
              //distance from robot to ball                      
          ballDistance = (double) Math.round(distanceCameraToBall*Math.sin(Math.toRadians(cameraAngle))/2)*2- robotDepth;
          
          // String Dsize = "Distance: " + ballDistance + "cm";
          int kat1 = cX-screenCenterX;
          int kat2 = cY-screenCenterY; 
          // SmartDashboard.putString("TestB: ", "0");
          // System.out.println("TestB");                     
        
          ballAngleX = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat1/(radius*ballDistance)))/2)*2;
          // ballAngleY = (double) Math.round(Math.toDegrees(Math.atan(ballRadius*kat2/(radius*ballDistance)))/5)*5;
          
//						new Point(x, y) 
          //showing angles and distance on the screen
          // String stringBallAngleX = ballAngleX + " X";
          // String stringBallAngleY =  ballAngleY + " Y "; 
          // Imgproc.putText(frame, Dsize, new Point(50, 50), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 2);	
          // String stringBallAngleX = ballAngleX + " X";
          // String stringBallAngleY =  ballAngleY + " Y "; 
          // String Dsize = ballDistance + "cm";
          // SmartDashboard.putString("Distance to ball: ", Dsize);
          // SmartDashboard.putString("Angle X: ", stringBallAngleX);
          // Imgproc.putText(frame, stringBallAngleX, new Point(20, 100), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);	
          // Imgproc.putText(frame, stringBallAngleY, new Point(20, 150), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);
          
          // Imgproc.putText(frame, Dsize, new Point(10, 20), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(255, 255, 0), 1);	
          // Imgproc.putText(frame, stringBallAngleX, new Point(20, 100), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);	
          // Imgproc.putText(frame, stringBallAngleY, new Point(20, 150), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 0), 4);
        }
    return thresh;
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
    
    m_autoSelected = m_chooser.getSelected();
    colorSelected = color_chooser.getSelected();
    speedStr = SmartDashboard.getString("Shooter Speed","0.7");
    leftEncoder = 96.52 *m_robotDrive.getLeftDistance();
    rightEncoder = -96.52*m_robotDrive.getRightDistance();
    
    // System.out.println("Drive: " + m_autoSelected);
    // System.out.println("Shooter Speed: " + speedStr);
    // SmartDashboard.putNumber("LEncoder", m_robotDrive.getLeftEncoder().getDistance());
    // SmartDashboard.putNumber("REncoder", m_robotDrive.getRightEncoder().getDistance());
    SmartDashboard.putNumber("Turn", m_robotDrive.getTurnRate());
    SmartDashboard.putNumber("LEncoder", leftEncoder);
    SmartDashboard.putNumber("REncoder", rightEncoder); 

    SmartDashboard.putNumber("Ball Distance", ballDistance);
    SmartDashboard.putNumber("Angle X", ballAngleX);
    SmartDashboard.putBoolean("IR 1 Readings", IRSensor1.get());
    SmartDashboard.putBoolean("IR 2 Readings", IRSensor2.get());  
    double x = 1;
    double y = 1;
    bAngleX.setDouble(x);
    bAngleY.setDouble(y);
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
       if (IRSensor1.get()==true || IRSensor2.get()==true)
       {
        //  runCANMechanism(conveyorMotor, 4, shootingPower, true);
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
    
    // try {
    //Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/YourPath.wpilib.json"));
    // }
    // catch (IOException e)
    // {}
    // ramsete1.initialize();
    // ramsete2.initialize();
    m_timer.reset();
    m_timer.start();
    m_robotDrive.resetEncoders();
    autoState = "shoot";
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // ramsete1.execute();
    // if (ramsete1.isFinished() && resetBool){
    //   resetBool = false;
    //   m_timer.reset();
    //   m_timer.start();
    // }
    // // DriverStation.reportWarning(Double.toString(m_timer.get()), true);
    // shooterMotor.set(0); 
    // if (m_timer.get() < 2.0 && !resetBool) {
    //   shooterMotor.set(0.7);
    // } 
    // else{
    // ramsete2.execute();
    // }

    // if (m_timer.get() < 2.0) {
    //      m_robotDrive.arcadeDrive(0, .65); // drive forwards half speed
    // }
    if (autoState == "firstDrive"){
      if (driveSetDistance(0.15, 0.7) == true){
        autoState = "shoot";
        m_timer.reset();
        m_timer.start();
        m_robotDrive.arcadeDrive(0, 0);
      }
    }
    else if (autoState == "shoot"){
      if (m_timer.get()<2.0){
        shooterMotor.set(0.65);
        conveyorMotor.set(0.7);
      }
      else {
        m_timer.reset();
        m_timer.start();
        shooterMotor.set(0);
        conveyorMotor.set(0);
        m_robotDrive.resetEncoders();
        autoState = "secondDrive";
      }
    }
    else if (autoState == "secondDrive"){
      // intakeSolenoid.set(true);
      // intakeMotor.set(ControlMode.PercentOutput, 0.7);
      // if (driveSetDistance(-1.3, -0.7) == true){
      //   autoState = "finished";
      // }
      if (m_timer.get()<2.0){
        m_robotDrive.arcadeDrive(0, -0.7);
      }
      else {
        autoState = "finished";
      }
    }
    else {//if (autoState == "finished")
      m_robotDrive.arcadeDrive(0, 0);
      
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  
  @Override
  public void teleopPeriodic() {
    if (tank){
    m_tTankDrive.execute();
    }
    shootingPower = Double.parseDouble(speedStr);
    // runShotAndIntake(7,8, shootingPower, nowState);
    runShotAndIntake(7,8, shootingPower, nowState);
    // motor, button, power
    // runCANMechanism(shooterMotor, 7, shootingPower, false, shootingPower/10);
    // runPneumaticCompressor(pcmCompressor, 2, true);
    // runPneumaticSolenoid(intakeSolenoid, 3, true);

    //establishes minimum and maximums of deadzone
    final double deadZone=0.4;
    final double minZone=0.07;
    final double invertAxis = -1;
    final double xOffset = 0.0;
    //positive xOffset goes right
    //gets joystick values and creates curves
    nowState = m_stick.getRawButton(3);
    double y = m_stick.getRawAxis(1);
    double yprime = invertAxis * Math.pow(y,3);
      // double yprime=-y;
    double x = m_stick.getRawAxis(2);
    double xprime = Math.pow(x,3);
      // double xprime=x;
    double accelFactor = 2.0;
    double slowFactor = 0.85;
    double speedFactor = 0.825;
    double turnFactor = 0.925;
    //The % power used
    final double turnLimit = 0.4;
    double speedLimit=0.5;
    final double leftAdj = 0;
    //Reports joystick numbers
    // DriverStation.reportWarning("Raw Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
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
    // DriverStation.reportWarning("New Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
    // R Bumper is 6
    if(m_stick.getRawButton(5)){
      yprime*=accelFactor;
    }
    
    if(m_stick.getRawButton(6)){
      yprime*=slowFactor;
      xprime*=slowFactor; 
    }
    xprime *= turnFactor;
    yprime *= speedFactor;
    //Actual drive part
    if (!tank && !driveModified){
      m_robotDrive.arcadeDrive(xprime+xOffset, yprime);
    }
  if (m_stick.getRawButton(10)){
    m_robotDrive.resetEncoders();
  }

  int encoderTarget = 20;
  int error = 2;
  // int minimumEncoder = encoderTarget - error;
  // int maximumEncoder = encoderTarget + error;
  
  double speed = 0.75;
  double margin = 5;
  double angle = ballAngleX;
  ballDistance*=0.01;
  if (prevState == false && nowState == true){
    m_robotDrive.resetEncoders();
  }
  prevState=m_stick.getRawButton(3);
  if (nowState){
      intakeSolenoid.set(nowState);
      intakeMotor.set(ControlMode.PercentOutput, 0.5);
      // DriverStation.reportWarning("ANGLE: "+Double.toString(angle),true);
      if (angle>0 && angle>margin){
        driveModified = true;
        // DriverStation.reportWarning("buton",true);
        m_robotDrive.arcadeDrive(speed, yprime);
      }
      else if (angle<0 && angle<margin){
        driveModified = true;
        // DriverStation.reportWarning("buton",true);
        m_robotDrive.arcadeDrive(-speed, yprime);
      }
      else if (Math.abs(angle) < margin){
        m_robotDrive.resetEncoders();
        if (leftEncoder<ballDistance && rightEncoder < ballDistance){
          m_robotDrive.arcadeDrive(0, -speed);
        }
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