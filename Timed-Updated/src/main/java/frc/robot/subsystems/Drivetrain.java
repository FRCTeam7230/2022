package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonToken;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DriveConstants;


public class Drivetrain {
    private DriveSubsystem m_robotDrive;
    private Joystick m_stick;
    private boolean swapState = false,prevState = false;
    private int invertAxis=-1;
    public Drivetrain(DriveSubsystem subsystem, Joystick stick){
        m_robotDrive = subsystem;
        m_stick = stick;
    }
    public void drive(boolean tank, boolean driveModified){
        final double deadZone=0.4;
        final double minZone=0.07;
        final double xOffset = 0.0;
        //positive xOffset goes right
        //gets joystick values and creates curves
        double y = m_stick.getRawAxis(1);
        double yprime = Math.pow(y,3);
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
        //Reports joystick numbers
        // DriverStation.reportWarning("Raw Y,X: "+((Double)yprime).toString()+","+((Double)xprime).toString(),true);
        //Mathmomagic! For X and Y 
        if(minZone<Math.abs(yprime)){
        // yprime=deadZone*Math.signum(yprime);
            yprime=(Math.abs(yprime)/yprime)*(deadZone+speedLimit*(1-deadZone)*(Math.abs(yprime)+deadZone)/(1))-deadZone/4;
            // DriverStation.reportWarning("BANANA"+((Double)Math.abs(yprime)).toString(),true);
        }
        else{
        yprime=0;
        }
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
            m_robotDrive.arcadeDrive(invertAxis * (xprime+xOffset), invertAxis *(yprime));
        }
        if (m_stick.getRawButton(10)){
            m_robotDrive.resetEncoders();
        }
        // ballDistance*=0.01;
        if (swapState == true && prevState == false){
            invertAxis *= -1;
            prevState = true;
        }
        else if (!swapState){
            prevState = false;
        }
    }
}