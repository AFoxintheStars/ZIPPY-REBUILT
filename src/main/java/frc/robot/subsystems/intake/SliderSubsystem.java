package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SliderSubsystem extends SubsystemBase
{
  private final String           motorTelemetryName = "SliderMotor";
  private final String           mechTelemetryName  = "SliderMechanism";
  private final SparkMax         leftSliderMotor      = new SparkMax(9, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax         rightSliderMotor     = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

  private final DCMotor          dcMotor            = DCMotor.getNEO(2);
  private final Distance         gearPitch         = Inches.of(1);
  private final int              toothCount         = 10;
  private final Distance         circumference      = gearPitch.times(toothCount);
  private final Distance         radius             = circumference.div(2 * Math.PI);
  private final MechanismGearing gearing            = new MechanismGearing(GearBox.fromReductionStages(4));
  private final Mass             weight             = Pounds.of(10);

  private final Distance            startingHeight      = Meters.of(0);

  private final Distance            softLowerLimit     = Meters.of(0);
  private final Distance            softUpperLimit     = Meters.of(0.3);

  private final Distance            hardLowerLimit     = Meters.of(0);
  private final Distance            hardUpperLimit     = Meters.of(0.3);

  private final ExponentialProfilePIDController pidController  = new ExponentialProfilePIDController(1,
                                                                                                     0,
                                                                                                     0,
                                                                                                     ExponentialProfilePIDController.createElevatorConstraints(
                                                                                                           Volts.of(12),
                                                                                                           dcMotor,
                                                                                                           weight,
                                                                                                           radius,
                                                                                                           gearing));

  private final ElevatorFeedforward             sliderFeedforward = new ElevatorFeedforward(0, 0, 0, 0);

  /**
  * {@link SmartMotorControllerConfig} for the elevator motor.
  */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(circumference)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withTelemetry(motorTelemetryName,
                     TelemetryVerbosity.HIGH)

      .withClosedLoopController(pidController)
      .withFeedforward(sliderFeedforward)
      .withSoftLimit(softLowerLimit, softUpperLimit)
      .withFollowers(Pair.of(rightSliderMotor, true));


  private final SmartMotorController motor         = new SparkWrapper(leftSliderMotor, dcMotor, motorConfig);

  private       ElevatorConfig       m_config      = new ElevatorConfig(motor)
      .withMass(weight)
      .withStartingHeight(startingHeight)
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      .withHardLimits(hardLowerLimit, hardUpperLimit);

  private final Elevator             m_slider    = new Elevator(m_config);

  public SliderSubsystem()
  {
  }

  public void periodic()
  {
    m_slider.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_slider.simIterate();
  }

  /**
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used when the Elevator
   * position is unreliable, like startup. Threshhold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Elevator is at it's hard limit.
   * @return
   */
  public Command homing(Current threshhold)
  {
      Debouncer       currentDebouncer  = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
      Voltage runVolts          = Volts.of(-2); // Volts required to run the mechanism down. Could be negative if the mechanism is inverted.
      Distance limitHit          = hardUpperLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
      AngularVelocity velocityThreshold = DegreesPerSecond.of(2); // The maximum amount of movement for the arm to be considered "hitting the hard limit".
      return Commands.startRun(motor::stopClosedLoopController, // Stop the closed loop controller
                      () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
              .until(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshhold) &&
                      motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                              velocityThreshold.in(DegreesPerSecond)))
              .finallyDo(() -> {
                  motor.setEncoderPosition(limitHit);
                  motor.startClosedLoopController();
              });
  }

  public Command sliderCmd (double dutycycle)
  {
    return m_slider.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return m_slider.setHeight(height);
  }
}
