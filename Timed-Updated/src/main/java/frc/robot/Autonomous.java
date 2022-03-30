package frc.robot;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
public class Autonomous {
  private Timer autonomousTimer = new Timer();
  private String autoState;
  public void init(DriveSubsystem subsystem){
    autonomousTimer.reset();
    autonomousTimer.start();
    subsystem.resetEncoders();
    autoState = "shoot";
  }
  public void execute(DriveSubsystem subsystem, CANSparkMax shooterMotor, CANSparkMax conveyorMotor, VictorSPX intakeMotor, Solenoid intakeSolenoid){
    if (autoState == "firstDrive"){
        if (Mechanisms.driveSetDistance(0.15, 0.7) == true){
          autoState = "shoot";
          autonomousTimer.reset();
          autonomousTimer.start();
          subsystem.arcadeDrive(0, 0);
        }
      }
      else if (autoState == "shoot"){
        if (autonomousTimer.get()<2.0){
          shooterMotor.set(0.7);
          conveyorMotor.set(0.7);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          shooterMotor.set(0);
  
          conveyorMotor.set(0);
          subsystem.resetEncoders();
          autoState = "secondDrive";
        }
      }
      else if (autoState == "secondDrive"){
        if (autonomousTimer.get()<2.0){
          subsystem.arcadeDrive(0, -0.7);
          intakeSolenoid.set(true);
          intakeMotor.set(ControlMode.PercentOutput, 0.65);
          conveyorMotor.set(0.5);
          shooterMotor.set(-0.1);
        }
        else {
          autonomousTimer.reset();
          autonomousTimer.start();
          intakeSolenoid.set(false);
          intakeMotor.set(ControlMode.PercentOutput, 0);
          conveyorMotor.set(0);
          shooterMotor.set(0);
          autoState = "finished";
        }
      }
      else {//if (autoState == "finished")
        subsystem.arcadeDrive(0, 0);
        intakeSolenoid.set(false);
        intakeMotor.set(ControlMode.PercentOutput, 0);
        conveyorMotor.set(0.5);
        shooterMotor.set(0);
        
      }
  }
}
