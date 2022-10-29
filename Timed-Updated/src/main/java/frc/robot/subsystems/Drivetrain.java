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
        //positive xOffset goes right
        //gets joystick values and creates curves
        double y = m_stick.getRawAxis(1);
        // double y = Math.pow(y,3);
        // double y=-y;
        double x = m_stick.getRawAxis(2);
        // double x = Math.pow(x,3);
        // double x=x;
        final double zoomFactor = 2.0;
        final double slowFactor = 0.85;
        final double speedFactor = 0.825;
        final double turnFactor = 0.925;

        double speedY = 0.0;
        double rateOfSpeedYChange = 0.0;
        double speedX = 0.0;
        double rateOfSpeedXChange = 0.0;
        final double accel = 0.05;
        if(deadZone < y && y > speedY) {
            speedY += rateOfSpeedYChange;
            rateOfSpeedYChange += accel;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
        }
        else if (deadZone>=y){
            speedY=0;
        }
        if(deadZone < x && x > speedX) {
            speedX += rateOfSpeedXChange;
            rateOfSpeedXChange += accel;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
        }
        else if (deadZone>=x){
            speedX=0;
        }

        // DriverStation.reportWarning("New Y,X: "+((Double)y).toString()+","+((Double)x).toString(),true);
        // R Bumper is 6
        if(m_stick.getRawButton(5)){
        y*=zoomFactor;
        }
        
        if(m_stick.getRawButton(6)){
        y*=slowFactor;
        x*=slowFactor; 
        }
        x *= turnFactor;
        y *= speedFactor;
        //Actual drive part
        if (!tank && !driveModified){
            m_robotDrive.arcadeDrive(invertAxis * (speedX), invertAxis *(speedY));
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