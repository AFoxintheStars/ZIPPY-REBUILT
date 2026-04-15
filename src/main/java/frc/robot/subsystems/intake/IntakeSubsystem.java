package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Simple, modular intake subsystem for running wheels.
 */
public class IntakeSubsystem extends SubsystemBase {
    
  public enum IntakeState {
    INTAKING,
    OUTTAKING,
    HOLDING,
    STOPPED
}

  private final SparkMax motor = new SparkMax(
    Constants.Intake.MOTOR_ID,
    SparkLowLevel.MotorType.kBrushless);

  private IntakeState currentState = IntakeState.STOPPED;

  private final SliderSubsystem slider;

  public IntakeSubsystem(SliderSubsystem slider) {
    this.slider = slider;
    configureMotor();
  }

  @SuppressWarnings("removal")
private void configureMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(Constants.Intake.INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) Constants.Intake.CURRENT_LIMIT.in(edu.wpi.first.units.Units.Amps));

    motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    motor.set(0);
  }

  // ===================== BASIC CONTROL =====================

  /** Run intake inward */
  public void intake() {
        if (slider.isInIntakeLockRange()) {
        stop();
        return;
    }

    currentState = IntakeState.INTAKING;
    motor.set(Constants.Intake.INTAKE_SPEED);
  }

  /** Run intake outward */
  public void outtake() {
    if (slider.isInIntakeLockRange()) {
        stop();
        return;
    }

    currentState = IntakeState.OUTTAKING;
    motor.set(Constants.Intake.OUTTAKE_SPEED);
  }

  /** Hold game piece gently */
  public void hold() {
    currentState = IntakeState.HOLDING;
    motor.set(Constants.Intake.HOLD_SPEED);
  }

  /** Stop motor */
  public void stop() {
    currentState = IntakeState.STOPPED;
    motor.stopMotor();
  }

  // ===================== COMMAND FACTORIES =====================

  public Command intakeCommand() {
    return Commands.startEnd(
        this::intake,
        this::stop,
        this);
  }

  public Command outtakeCommand() {
    return Commands.startEnd(
        this::outtake,
        this::stop,
        this);
  }

  public Command holdCommand() {
    return Commands.startEnd(
        this::hold,
        this::stop,
        this);
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  // ===================== EXTENSIONS =====================

  /**
   * Detect stall based on current
   */
  public boolean isStalling() {
    return motor.getOutputCurrent() > Constants.Intake.CURRENT_LIMIT.in(edu.wpi.first.units.Units.Amps);
  }

  @Override
  public void periodic() {

    if (slider.isInIntakeLockRange() && currentState == IntakeState.INTAKING) {
      stop();
    }

    SmartDashboard.putNumber("Intake/Output", motor.get());
    SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());
    SmartDashboard.putBoolean("Intake/HasPiece", isStalling());
    SmartDashboard.putString("Intake/State", currentState.name());
    SmartDashboard.putBoolean("Intake/LockedBySlider", slider.isInIntakeLockRange());
  }
}