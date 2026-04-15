// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SliderSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withMechanismCircumference(Meters.of(Inches.of(1).in(Meters) * 10))
    .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.2), MetersPerSecondPerSecond.of(0.1))
    .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withTelemetry("SliderMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.20))
    .withOpenLoopRampRate(Seconds.of(0.20))
    .withFollowers(Pair.of(new SparkMax(10, MotorType.kBrushless), true));

  private SparkMax spark = new SparkMax(9, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(2), smcConfig);

  private ElevatorConfig sliderconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Meters.of(0))
      .withHardLimits(Meters.of(0), Meters.of(0.3))
      .withTelemetry("Slider", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(10));

  private Elevator slider = new Elevator(sliderconfig);

  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param height Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return slider.run(height);}
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param height Distance to go to.
   * @param tolerance Distance tolerance for completion.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height, Distance tolerance) { return slider.runTo(height, tolerance);}
  
  /**
   * Set the elevators closed loop controller setpoint.
   * @param height Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { slider.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return slider.set(dutycycle);}

  public boolean isInIntakeLockRange() {
    double height = slider.getHeight().in(Meters);

    return height >= Constants.Slider.INTAKE_LOCK_MIN &&
           height <= Constants.Slider.INTAKE_LOCK_MAX;
  }

  public SliderSubsystem() {}

  @Override
  public void periodic() {
    slider.updateTelemetry();
    SmartDashboard.putNumber("Slider Distance (m)", slider.getHeight().in(Meters));
  } 

  @Override
  public void simulationPeriodic() {
    slider.simIterate();
  }
}
