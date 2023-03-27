// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    //public static final int kDriverControllerPort = 0;

    //ARM and PIVOT motor config constants

  }

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;

  //Grabber Constants

  public static final int SOLENOID_ID1 = 0;
  public static final int SOLENOID_ID2 = 1;


  //Vision Constants
  public static final String USB_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";


  //CAN IDS
  public static final int LEFT_FRONT = 1;
  public static final int LEFT_BACK = 2;
  public static final int RIGHT_FRONT = 3;
  public static final int RIGHT_BACK = 4;
  public static final int PIGEON_CANID =6;
  public static final int ARM_MOTOR = 5;
  public static final int PIVOT_MOTOR = 7;


  //DRIVETRAIN CONSTANTS
  public static final double DRIVE_CRUISE_VELOCITY = 6000;
  public static final double DRIVE_ACCELERATION = 2000;
  public static final int DRIVE_PID_TIMEOUT = 30;
  public static final double BALANCE_GOAL_DEGREE = 0;
  public static final double BALANCE_REVERSE_POWER = 1.3;


  public static final double TRACKED_TAG_AREA_KP = 0.2;
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28.0);
  public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18.0);;
  public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3;
  public static final double TRACK_TAG_ROTATION_KP = 0.0175;
  public static final double APRILTAG_POWER_CAP = 0.75;


  public static final int DRIVER_CONTROLLER = 0;
  public static final int EXTREME_CONTROLLER = 1;

  //MANUAL CONTROL POWER VALUES
  public static final double MANUAL_ARM_POWER = 0.6;
  public static final double MANUAL_PIVOT_POWER = 0.1;
public static final int POT_PORT = 0;




}
