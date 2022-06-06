package frc.robot;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  private DriveSubsystem m_dDriveSubsystem;
  private Mechanisms mechanisms;
  private CANSparkMax shooter, conveyor;
  private VictorSPX intake;
  private Solenoid intakeSol;
  
  public Autonomous(DriveSubsystem subsystem, CANSparkMax shooterMotor, CANSparkMax conveyorMotor, VictorSPX intakeMotor, Solenoid intakeSolenoid){

    m_dDriveSubsystem=subsystem;
    m_dDriveSubsystem.resetEncoders();
    shooter = shooterMotor;
    conveyor = conveyorMotor;
    intake = intakeMotor;
    intakeSol = intakeSolenoid;
    autoState = "shoot";
  }
  public void init(){
    m_dDriveSubsystem.resetEncoders();    
    autonomousTimer.reset();
    autonomousTimer.start();
  }
  public void execute(){
    if (autoState == "firstDrive"){
        if (mechanisms.driveSetDistance(0.15, 0.7) == true){
          autoState = "shoot";
          autonomousTimer.reset();
          autonomousTimer.start();
          m_dDriveSubsystem.arcadeDrive(0, 0);
        }
      }
      else if (autoState == "shoot"){
        if (autonomousTimer.get()<2.0){
          shooter.set(0.7);
          conveyor.set(0.7);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          shooter.set(0);
  
          conveyor.set(0);
          m_dDriveSubsystem.resetEncoders();
          autoState = "secondDrive";
        }
      }
      else if (autoState == "secondDrive"){
        if (autonomousTimer.get()<2.0){
          m_dDriveSubsystem.arcadeDrive(0, -0.7);
          intakeSol.set(true);
          intake.set(ControlMode.PercentOutput, 0.65);
          conveyor.set(0.5);
          shooter.set(-0.1);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          intakeSol.set(false);
          intake.set(ControlMode.PercentOutput, 0);
          conveyor.set(0);
          shooter.set(0);
          autoState = "finished";
        }
      }
      else {//if (autoState == "finished")
        m_dDriveSubsystem.arcadeDrive(0, 0);
        intakeSol.set(false);
        intake.set(ControlMode.PercentOutput, 0);
        conveyor.set(0.5);
        shooter.set(0);
        
      }
  }
}
