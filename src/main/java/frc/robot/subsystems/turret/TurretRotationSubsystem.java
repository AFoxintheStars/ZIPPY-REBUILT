package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.TurretConstants;

public class TurretRotationSubsystem extends SubsystemBase {

    /* ==================== HARDWARE ==================== */

    private final SparkMax turretMotor =
        new SparkMax(TurretConstants.MOTOR_ID, MotorType.kBrushless);

    public final RelativeEncoder encoder = turretMotor.getEncoder();

    /* ==================== CONSTRUCTOR ==================== */

    public TurretRotationSubsystem() {
        configureMotor();
    }

    @SuppressWarnings("removal")
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(TurretConstants.CURRENT_LIMIT)
            .inverted(TurretConstants.INVERTED);

        config.encoder
            .positionConversionFactor(TurretConstants.DEGREES_PER_MOTOR_ROTATION)
            .velocityConversionFactor(TurretConstants.DEGREES_PER_MOTOR_ROTATION / 60.0);

        turretMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        zeroTurret();
    }

    /* ==================== CONTROL ==================== */

    public void setSpeed(double speed) {
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    public void zeroTurret() {
        encoder.setPosition(0);
    }

    public boolean atLeftLimit() {
    return getAngleDegrees() <= TurretConstants.MIN_ANGLE;
    }

    public boolean atRightLimit() {
        return getAngleDegrees() >= TurretConstants.MAX_ANGLE;
    }

    /* ==================== ANGLE ==================== */

    public double getAngleDegrees() {
        return encoder.getPosition();
    }

    public double getVelocityDegPerSec() {
        return encoder.getVelocity();
    }

    /* ==================== COMMANDS ==================== */

    public Command rotateManual(double speed) {
        return Commands.startEnd(
            () -> setSpeed(speed),
            this::stop,
            this
        );
    }

    public Command rotateLeft() {
        return rotateManual(TurretConstants.MANUAL_SPEED);
    }

    public Command rotateRight() {
        return rotateManual(-TurretConstants.MANUAL_SPEED);
    }

    /* ==================== TELEMETRY ==================== */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle (deg)", getAngleDegrees());
        SmartDashboard.putNumber("Turret/Velocity (deg/s)", getVelocityDegPerSec());
        SmartDashboard.putBoolean("Turret/At Left Limit", atLeftLimit());
        SmartDashboard.putBoolean("Turret/At Right Limit", atRightLimit());
    }
}