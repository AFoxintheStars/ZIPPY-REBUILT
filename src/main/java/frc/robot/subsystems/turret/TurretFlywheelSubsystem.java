package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.FlywheelConstants;
    
public class TurretFlywheelSubsystem extends SubsystemBase {

    /* ==================== HARDWARE ==================== */

    private final SparkMax flywheelMotor =
        new SparkMax(FlywheelConstants.MOTOR_ID, MotorType.kBrushless);

    private final SparkMax follower =
        new SparkMax(FlywheelConstants.FOLLOWER_ID, MotorType.kBrushless);

    private final RelativeEncoder encoder = flywheelMotor.getEncoder();

    private final SparkClosedLoopController controller =
        flywheelMotor.getClosedLoopController();

    /* ==================== STATE ==================== */

    private double targetRPM = 0;

    /* ==================== CONSTRUCTOR ==================== */
    public TurretFlywheelSubsystem() {
        configureMotor();
    }

    @SuppressWarnings("removal")
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(FlywheelConstants.IDLE_MODE)
            .smartCurrentLimit(FlywheelConstants.CURRENT_LIMIT)
            .inverted(FlywheelConstants.INVERTED);

        config.encoder
            .velocityConversionFactor(1.0);

        config.closedLoop
            .pid(
                FlywheelConstants.kP,
                FlywheelConstants.kI,
                FlywheelConstants.kD
            )
            .velocityFF(FlywheelConstants.kFF)
            .outputRange(-1.0, 1.0);

        flywheelMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        follower.configure(
            new SparkMaxConfig().follow(flywheelMotor, FlywheelConstants.FOLLOWER_INVERTED),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /* ==================== CONTROL ==================== */

    public void setRPM(double flywheelRPM) {
        targetRPM = flywheelRPM;

        double motorRPM = flywheelRPM * FlywheelConstants.GEAR_RATIO;

        controller.setSetpoint(motorRPM, ControlType.kVelocity);
    }

    public void adjustRPM(double delta) {
        double newRPM = targetRPM + delta;

        newRPM = Math.max(
            FlywheelConstants.MIN_RPM,
            Math.min(FlywheelConstants.MAX_RPM, newRPM)
        );

        setRPM(newRPM);
    }

    public void maintainRPM() {
        controller.setSetpoint(
            targetRPM * FlywheelConstants.GEAR_RATIO,
            ControlType.kVelocity
        );
    }

    public boolean isReady() {
        return Math.abs(getRPM() - targetRPM)
            <= FlywheelConstants.RPM_TOLERANCE;
    }

    public void stop() {
        targetRPM = 0;
        flywheelMotor.stopMotor();
    }

    /* ==================== TELEMETRY ==================== */

    public double getRPM() {
        return encoder.getVelocity() / FlywheelConstants.GEAR_RATIO;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean atTargetSpeed() {
        return Math.abs(getRPM() - targetRPM) <= FlywheelConstants.RPM_TOLERANCE;
    }

    public double getCurrent() {
        return flywheelMotor.getOutputCurrent();
    }

    /* ==================== COMMANDS ==================== */

    public Command spinUp(double rpm) {
        return Commands.startEnd(
            () -> setRPM(rpm),
            this::stop,
            this
        );
    }

    /* ==================== PERIODIC ==================== */

    @Override
    public void periodic() {
    if (targetRPM > 0) {
        maintainRPM();
    }

    SmartDashboard.putNumber("Flywheel/RPM", getRPM() * 2);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
    SmartDashboard.putNumber("Flywheel/Error", targetRPM - getRPM());
    SmartDashboard.putBoolean("Flywheel/At Speed", atTargetSpeed());
    SmartDashboard.putNumber("FLywheel/Current", getCurrent());
    }
}