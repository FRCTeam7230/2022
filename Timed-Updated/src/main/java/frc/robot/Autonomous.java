package frc.robot;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private DriveSubsystem m_dDriveSubsystem;
  private Drivetrain driveTrain;
  private Mechanisms mechanisms;
  private CANSparkMax shooter, conveyor;
  private VictorSPX intake;
  private Solenoid intakeSol;
  private boolean search4Ball = true;
  private double oldDistance4Ball = 0;
  private boolean driveModified = false;  
  private Joystick m_stick;
  public boolean ballInIntake = true; //if the robot took the ball - true, did not take - fall

  public Autonomous(DriveSubsystem subsystem, CANSparkMax shooterMotor, CANSparkMax conveyorMotor, VictorSPX intakeMotor, Solenoid intakeSolenoid){
    m_dDriveSubsystem = subsystem;
    m_dDriveSubsystem.resetEncoders();
    shooter = shooterMotor;
    conveyor = conveyorMotor;
    intake = intakeMotor;
    intakeSol = intakeSolenoid;
    autoState = "firstDrive";
  }
  public void init(){
    m_dDriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
  }
  public void execute(double ballDistance, double ballAngleX){

    // First phase - drive - drive to the hub and shoot ball
    // Second phase - drive backwards
    // Third phase - findNewBall - turn around, find ball
    // Forth phase - grabBall - drive toward the ball and grab it then

    if (!m_stick.getRawButton(4)) { //emergency button Y

      // if (ballInIntake) {
      //    conveyor.set(0.5);
      // }

      // Phase 1 - Move forward to the hub
      if (autoState == "firstDrive") { 
        if (mechanisms.driveSetDistance(0.15, 0.5)){ 
            m_dDriveSubsystem.arcadeDrive(0, 0);
            autonomousTimer.reset();
            autonomousTimer.start();
        }
        else {
          shootBall();
          autoState = "secondDrive";
        }
      } 

      //Phase 2 - Drive backwards - do we need intake and conveyor running? To catch random balls
      else if (autoState == "secondDrive"){ 
        if (autonomousTimer.get() < 2.0){
          m_dDriveSubsystem.arcadeDrive(0, -0.7);
          // intakeSol.set(true);
          // intake.set(ControlMode.PercentOutput, 0.65);
          // conveyor.set(0.5);
          // shooter.set(-0.1);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          // intakeSol.set(false);
          // intake.set(ControlMode.PercentOutput, 0);
          // conveyor.set(0);
          // shooter.set(0);
          autoState = "findNewBall";
        }
      }

      // Phase 3 - Turn around and check where is the ball
      else if (autoState == "findNewBall") {

        // Phase 4 - Drive towards the ball and grab it
        if (search4Ball) { // Minuses - if the ball is swinging, if Robot  detected another colored object, not the ball
          m_dDriveSubsystem.drive(0.2, -0.2);

          if ((autoState == "grabBall") || (autonomousTimer.get() > 0.1 && oldDistance4Ball/10 == ballDistance/10)) {
            autoState = "grabBall";
            m_dDriveSubsystem.drive(0,0);
            
            
            intakeSol.set(true);
            intake.set(ControlMode.PercentOutput, 0.65);
            conveyor.set(0.5);
            driveTrain.drive(true, driveModified);

            double speed = 0.7;
            double margin = 3;
            double angle = ballAngleX;

            if (angle>0 && angle>margin){ // Find right direction
              driveModified = true;
              m_dDriveSubsystem.drive(speed/4, -speed/2);
            }
            else if (angle<0 && angle<-margin){
              driveModified = true;
              m_dDriveSubsystem.drive(-speed/2, speed/4);
            }
            else if (Math.abs(angle) < margin){
              driveModified = true;
              m_dDriveSubsystem.arcadeDrive(0, -speed/2);
            }

            ballInIntake = true;
          }
          
          else {
            oldDistance4Ball = ballDistance;
            autonomousTimer.reset();
            autonomousTimer.start();
            System.out.println("Three phases completed. Waiting....");
          }
        }
        // Phase 5 - receive information from encoders - calculate the distance that the robot has traveled. 
        //Also, add code for "remembering" the angle of rotation of the robot - maybe encoder too.
        // Or, when moving to grab the ball, "remeber" the change in position (which direction they turned and how long) of left and right wheels - just turn in opposite direction
      }
    }
    else { 
      System.out.println("Waiting....");
    }
  }
  public void shootBall() {
    if (autonomousTimer.get() < 2.0) { // Run the shooter & conveyor for 2s
      conveyor.set(0.5); 
      shooter.set(0.5);
    } 
    else { // Reset values and stop shooter & conveyor
      autonomousTimer.reset();
      autonomousTimer.start();
      shooter.set(0);
      conveyor.set(0);
      ballInIntake = false;
    }
  }
}

/*
        else if (autonomousTimer.get() >= 2 && autonomousTimer.get() <= 4) { // drive backwards 2s with intake and conveyor running
            if (shooter.get() != 0) {
                shooter.set(0);
            }
            m_DriveSubsystem.arcadeDrive(0, -0.5);
            intakeSol.set(true);
            intake.set(ControlMode.PercentOutput, 0.5);
            conveyor.set(0.5);
        } 
        else { // stop the vehicle & conveyor running upwards slowly
            m_DriveSubsystem.arcadeDrive(0, 0);
            conveyor.set(0.2);
            intakeSol.set(false);
            intake.set(ControlMode.PercentOutput, 0);
        }
    }
}  */
