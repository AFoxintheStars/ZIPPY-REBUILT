// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;
import swervelib.math.Matter;

public final class Constants
{

public static final double ROBOT_MASS = 100 * 0.453592;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13;
  public static final double MAX_SPEED  = Units.feetToMeters(15);

  public static final class DrivebaseConstants
  {
  }

  public static final class Intake 
{
    public static final int MOTOR_ID = 11;

    public static final double INTAKE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED = -0.8;
    public static final double HOLD_SPEED = 0.1;

    public static final Current CURRENT_LIMIT = Amps.of(40);

    public static final boolean INVERTED = false;
  }

  public static final class Prefeed
  {
    public static final int LEADER_ID = 12;
    public static final int FOLLOWER_LEFT_ID = 13;
    public static final int FOLLOWER_RIGHT_ID = 14;

    public static final boolean LEADER_INVERTED = false;
    public static final boolean FOLLOWER_LEFT_INVERTED = false;
    public static final boolean FOLLOWER_RIGHT_INVERTED = false;

    public static final int CURRENT_LIMIT = 40;

    public static final double INTAKE_SPEED = 0.7;
    public static final double OUTTAKE_SPEED = -0.6;

    public static final double STALL_CURRENT_THRESHOLD = 35.0;
    public static final double STALL_VELOCITY_THRESHOLD = 100;
  }

  public static final class TurretConstants 
  {

    public static final int MOTOR_ID = 15;

    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 30;

    // Gear ratio: 200:16
    public static final double GEAR_RATIO = 12.5;

    public static final double DEGREES_PER_MOTOR_ROTATION = 360.0 / GEAR_RATIO;

    public static final double MIN_ANGLE = -90;
    public static final double MAX_ANGLE = 90;

    public static final double MANUAL_SPEED = 0.1;
}

public static final class FlywheelConstants 
{

    public static final int MOTOR_ID = 16;

    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 40;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final double kP = 0.0002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.00017;

    public static final double SHOOT_RPM = 20000;

    public static final double RPM_TOLERANCE = 100;
    
    public static final int READY_CYCLES = 5;
}

  public static class OperatorConstants
  {
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}