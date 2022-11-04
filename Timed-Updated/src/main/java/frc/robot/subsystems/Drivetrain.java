package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonToken;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
    public Drivetrain(DriveSubsystem subsystem, Joystick stick){
        m_robotDrive = subsystem;
        m_stick = stick;
    }
    public void drive(boolean tank, boolean driveModified){
        double y = m_stick.getRawAxis(2);
        double x = m_stick.getRawAxis(1);
        int invertChangeY = 1;
        if (y<0){
            invertChangeY = -1;
        }
        int invertChangeX = 1;
        if (y<0){
            invertChangeX = -1;
        }
        if(driveTrainConstants.deadZone < y && y > speedY) {
            speedY += invertChangeY * rateOfSpeedYChange;
            rateOfSpeedYChange += driveTrainConstants.accelY;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
        }
        else if (driveTrainConstants.deadZone>=y){
            speedY=0;
        }
        if(driveTrainConstants.deadZone < x && x > speedX) {
            speedX += invertChangeX * rateOfSpeedXChange;
            rateOfSpeedXChange += driveTrainConstants.accelX;
            //Quadratic Rate of Change if I think
            //y goes forward and back
            //replace ys below this with speed?
        }
        else if (driveTrainConstants.deadZone>=x){
            speedX=0;
        }

        // DriverStation.reportWarning("New Y,X: "+((Double)y).toString()+","+((Double)x).toString(),true);
        // R Bumper is 6
        if(m_stick.getRawButton(robotConstants.L_BUMPER)){
        y*=driveTrainConstants.zoomFactor;
        }
        
        if(m_stick.getRawButton(robotConstants.R_BUMPER)){
        y*=driveTrainConstants.slowFactor;
        x*=driveTrainConstants.slowFactor; 
        }
        x *= driveTrainConstants.turnFactor;
        y *= driveTrainConstants.speedFactor;
        speedX = Math.min(speedX, 0.9);
        speedY = Math.min(speedY, 0.9);
        //Actual drive part
        if (!tank && !driveModified){
            DriverStation.reportWarning(Double.toString(x), false);
            DriverStation.reportWarning(Double.toString(y), false);
            // DriverStation.reportWarning(Double.toString(speedX), false);
            // DriverStation.reportWarning(Double.toString(speedY), false);
            m_robotDrive.arcadeDrive(invertAxis * (speedY), invertAxis *(speedX));
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
    }
}