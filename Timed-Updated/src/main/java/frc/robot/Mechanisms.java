package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
public class Mechanisms {
    private DriveSubsystem m_DriveSubsystem;
    private CANSparkMax shooterMotor, conveyorMotor; 
    private VictorSPX intakeMotor;
    private Solenoid intakeSolenoid;
    private Joystick m_stick;
    private double leftEncoder, rightEncoder;
    private boolean previousState = false;
    private Timer climbTimer = new Timer();
    private Timer shotTimer = new Timer();
    public Mechanisms(Joystick stick, DriveSubsystem subsystem, CANSparkMax shooter, CANSparkMax conveyor, VictorSPX intake, Solenoid intakeSol){
        m_stick = stick;
        m_DriveSubsystem = subsystem;
        shooterMotor = shooter;
        conveyorMotor = conveyor;
        intakeMotor = intake;
        intakeSolenoid = intakeSol;

    }
    
   public void runCANMechanism(CANSparkMax motor, int button, double power, boolean invert, double offPower){
    boolean state = m_stick.getRawButton(button); 
    double newPower = power;
     if (invert){
       newPower*=-1;
     }
     if(state){
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
    climbTimer.reset();
    climbTimer.start();
    if (climbTimer.get() < 4.0){
      solenoid.set(true);
      if (climbTimer.get()>1.0){
        motor.set(speed);
      }
    }
    if (climbTimer.get() > 4.0 && climbTimer.get() < 8.0){
      solenoid.set(false);
      if (climbTimer.get()>5.0 && climbTimer.get() < 8.0){
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
      if (previousState == false && state1 == true){
        shotTimer.reset();
        shotTimer.start();
      }
      previousState = state1;
      if(state1){
        // DriverStation.reportWarning("running button "+button1+"fwd",true);
        if (shotTimer.get()<0.0625){
          shooterMotor.set(0);
          conveyorMotor.set(-0.5);
        }
        else if (shotTimer.get()>=0.0625 &&  shotTimer.get()<0.5625){
          shooterMotor.set(newPower);
          conveyorMotor.set(0);
        }
        else if (shotTimer.get()>=0.5625){
          conveyorMotor.set(0.5);
          shooterMotor.set(newPower);
        }
        // conveyorMotor.set(0.5);
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
      runPneumaticSolenoid(intakeSolenoid, button2, enabled);
      runSPXMechanism(intakeMotor, button2, 0.65, false);
  }

  public boolean driveSetDistance(double distance, double speed){
    boolean finished = false;
    leftEncoder = 96.52 *m_DriveSubsystem.getLeftDistance();
    rightEncoder = -96.52*m_DriveSubsystem.getRightDistance();
    if (Math.abs(leftEncoder)<Math.abs(distance) && Math.abs(rightEncoder) < Math.abs(distance)){
      m_DriveSubsystem.arcadeDrive(0, speed);
      // DriverStation.reportWarning(Double.toString(leftEncoder) + ", "+ Double.toString(distance), true);
      
    }
    else {
      finished = true;
    }
    return finished;
  }
}
