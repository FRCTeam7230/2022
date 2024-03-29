/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class robotConstants {
    public static final int L1MOTOR_ID = 1;
    public static final int R1MOTOR_ID = 3;
    public static final int L2MOTOR_ID = 2;
    public static final int R2MOTOR_ID = 4;
    
    // Documentation of joystick.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html

    public static final int BUTTON1 = 1; //X BUTTON: CLIMBER UP
    public static final int BUTTON2 = 2; //A BUTTON: CLIMBER DOWN
    public static final int BUTTON3 = 3; //B BUTTON: SMART VISION INTAKE
    public static final int BUTTON4 = 4; //Y BUTTON: REVERSE
    public static final int BUTTON5 = 5; //L BUMPER: SPEED
    public static final int BUTTON6 = 6; //R BUMPER: SLOW
    public static final int BUTTON7 = 7; //L TRIGGER: SHOOT
    public static final int BUTTON8 = 8; //R TRIGGER: INTAKE
    public static final int BUTTON9 = 9; 
    public static final int BUTTON10 = 10; //START BUTTON: RUN INTAKE MOTOR/NO SOLENOID
    public static final int BUTTON11 = 11;
    public static final int BUTTON12 = 12;


  }

  public static final class driveTrainConstants {
    public static final double deadZone=0.3;
    public static final double zoomFactor = 2.0;
    public static final double slowFactor = 0.45;
    public static final double speedFactor = 0.775;
    public static final double turnFactor = 0.725;
    public static final double accelY = 0.005;
    public static final double accelX = 0.001;
    
  }


  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[]{2, 3};
    public static final int[] kRightEncoderPorts = new int[]{0, 1};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.55;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 2.14;
    public static final double kvVoltSecondsPerMeter = 1.69;
    public static final double kaVoltSecondsSquaredPerMeter = 0.294;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 1.8;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}