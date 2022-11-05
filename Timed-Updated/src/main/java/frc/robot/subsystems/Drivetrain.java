package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonToken;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.Constants.robotConstants;

public class Drivetrain {
    private DriveSubsystem m_robotDrive;
    private Joystick m_stick;
    private boolean swapState = false,prevState = false;
    private int invertAxis=-1;
    double speedY = 0.0;
    double speedX = 0.0;
    double rateOfSpeedYChange = 0.0;
    double rateOfSpeedXChange = 0.0;
    boolean prevDrive = false, nowDrive = false;
    public Drivetrain(DriveSubsystem subsystem, Joystick stick){
        m_robotDrive = subsystem;
        m_stick = stick;
    }
    public void drive(boolean tank, boolean driveModified){
        double y = Math.pow(m_stick.getRawAxis(2),1);
        double x = Math.pow(m_stick.getRawAxis(1),1);
        y *= Math.abs(y);
        x *= Math.abs(x);
        int invertChangeY = 1;
        int invertChangeX = 1;
        // System.out.println(y + " " + speedY);
        if (y< -1*driveTrainConstants.deadZone || (speedY<0 && Math.abs(y)<driveTrainConstants.deadZone)){
            invertChangeY = -1;
        }
        if (x<-1*driveTrainConstants.deadZone || (speedX<0 && Math.abs(x)<driveTrainConstants.deadZone)){
            invertChangeX = -1;
        }
        if(driveTrainConstants.deadZone < Math.abs(y) && Math.abs(y) > Math.abs(speedY)) {
            speedY += invertChangeY * rateOfSpeedYChange;
            rateOfSpeedYChange += driveTrainConstants.accelY;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
            nowDrive = true;
        }
        else if (driveTrainConstants.deadZone>=Math.abs(y) && Math.abs(speedY)<0.4){
            speedY=0;
        }
        else if (driveTrainConstants.deadZone>=Math.abs(y) && Math.abs(speedY)>=0.4){
            speedY-=0.075*invertChangeY;
            // System.out.println("ASFJLKJ" + invertChangeY);
            // DriverStation.reportWarning("warning", false);
        }
        if(driveTrainConstants.deadZone < Math.abs(x) && Math.abs(x) > Math.abs(speedX)) {
            speedX += invertChangeX * rateOfSpeedXChange;
            rateOfSpeedXChange += driveTrainConstants.accelX;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
            nowDrive = true;
            
        }
        else if (driveTrainConstants.deadZone>=Math.abs(x) && Math.abs(speedX)<0.4){
            speedX=0;
        }
        else if (driveTrainConstants.deadZone>=Math.abs(x) && Math.abs(speedX)>=0.4){
            speedX-=0.075*invertChangeX;
        }
        if (Math.abs(x)<driveTrainConstants.deadZone && Math.abs(y)<driveTrainConstants.deadZone)
        {
            nowDrive = false;
        }
        if (nowDrive && !prevDrive){
            rateOfSpeedXChange = 0;
            rateOfSpeedYChange = 0;
            if (Math.abs(x)>driveTrainConstants.deadZone){
                speedX+=0.2 * invertChangeX;
            }
            if (Math.abs(y)>driveTrainConstants.deadZone){
                speedY+=0.2 * invertChangeY;
            }
        }
        // DriverStation.reportWarning("New Y,X: "+((Double)y).toString()+","+((Double)x).toString(),true);
        // R Bumper is 6
        if(m_stick.getRawButton(robotConstants.L_BUMPER)){
            speedY*=driveTrainConstants.zoomFactor;
        }
        
        if(m_stick.getRawButton(robotConstants.R_BUMPER)){
            speedX=x;
            speedY = y;
            speedY*=driveTrainConstants.slowFactor;
            speedX*=driveTrainConstants.slowFactor; 
        }
        // x *= driveTrainConstants.turnFactor;
        // y *= driveTrainConstants.speedFactor;
        // IMPORTANT
        // I DONT KNOW WHY BUT X AND Y LIMITS HERE ARE SWITCHED
        if (speedX > 0){
            speedX = Math.min(speedX, 0.8);
        }
        else {
            speedX = Math.max(speedX, -0.8);
        }
        if (speedY > 0){
            speedY = Math.min(speedY, 0.6);
        }
        else {
            speedY = Math.max(speedY, -0.6);
        }
        //Actual drive part
        SmartDashboard.putNumber("DRIVE X", speedX);
        SmartDashboard.putNumber("DRIVE Y", speedY);
        if (!tank && !driveModified){
            // DriverStation.reportWarning(Double.toString(x), false);
            // DriverStation.reportWarning(Double.toString(y), false);
            // DriverStation.reportWarning(Double.toString(speedX), false);
            DriverStation.reportWarning(Double.toString(speedY), false);
            m_robotDrive.arcadeDrive(-1 * invertAxis * (speedY), invertAxis *(speedX));
        }
        // if (m_stick.getRawButton(robotConstants.START_BUTTON)){
        //     m_robotDrive.resetEncoders();
        // }
        // ballDistance*=0.01;
        swapState = m_stick.getRawButton(robotConstants.Y_BUTTON);
        if (swapState == true && prevState == false){
            invertAxis *= -1;
            prevState = true;
        }
        else if (!swapState){
            prevState = false;
        }
        prevState = swapState;
        prevDrive = nowDrive;
    }
}