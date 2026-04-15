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
import frc.robot.subsystems.prefeed.PrefeedSubsystem;
    
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

    private int atSpeedCycles = 0;

    /* ==================== CONSTRUCTOR ==================== */

    @SuppressWarnings("removal")
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

    public boolean isReadyToShoot() {
    if (Math.abs(getRPM() - targetRPM) <= FlywheelConstants.RPM_TOLERANCE) {
        atSpeedCycles++;
    } else {
        atSpeedCycles = 0;
    }

    return atSpeedCycles >= FlywheelConstants.READY_CYCLES;
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

    public Command shoot() {
        return Commands.startEnd(
            () -> setRPM(FlywheelConstants.SHOOT_RPM),
            this::stop,
            this
        );
    }

    public Command spinUpAndWait(double rpm) {
    return Commands.sequence(
        Commands.runOnce(() -> setRPM(rpm)),
        Commands.waitUntil(this::isReadyToShoot)
        );
    }

    public Command shootWithPrefeed(PrefeedSubsystem prefeed) {
    return Commands.parallel(
        shoot(),
        prefeed.intake().onlyIf(this::isReadyToShoot)
        );
    }

    /* ==================== PERIODIC ==================== */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/RPM", getRPM());
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/Error", targetRPM - getRPM());
        SmartDashboard.putBoolean("Flywheel/At Speed", atTargetSpeed());
        SmartDashboard.putBoolean("Flywheel/Ready", isReadyToShoot());
        SmartDashboard.putBoolean("Flywheel/Spinning Up", !isReadyToShoot());
         SmartDashboard.putNumber("FLywheel/Current", getCurrent());
    }
}