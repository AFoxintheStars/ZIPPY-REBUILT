package frc.robot.subsystems.prefeed;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.Prefeed;

public class PrefeedSubsystem extends SubsystemBase {

    /* ==================== MOTORS ==================== */

    private final SparkMax leader =
        new SparkMax(Prefeed.LEADER_ID, MotorType.kBrushless);

    private final SparkMax followerLeft =
        new SparkMax(Prefeed.FOLLOWER_LEFT_ID, MotorType.kBrushless);

    private final SparkMax followerRight =
        new SparkMax(Prefeed.FOLLOWER_RIGHT_ID, MotorType.kBrushless);

    /* ==================== CONSTRUCTOR ==================== */

    @SuppressWarnings("removal")
    public PrefeedSubsystem() {
        configureMotors();
    }

    @SuppressWarnings("removal")
    private void configureMotors() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();

        baseConfig
            .smartCurrentLimit(Prefeed.CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast);

        leader.configure(
            new SparkMaxConfig()
                .apply(baseConfig)
                .inverted(Prefeed.LEADER_INVERTED),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        followerLeft.configure(
            new SparkMaxConfig()
                .apply(baseConfig)
                .follow(leader, Prefeed.FOLLOWER_LEFT_INVERTED),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        followerRight.configure(
            new SparkMaxConfig()
                .apply(baseConfig)
                .follow(leader, Prefeed.FOLLOWER_RIGHT_INVERTED),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /* ==================== BASIC CONTROL ==================== */

    public void setSpeed(double speed) {
        leader.set(speed);
    }

    public void stop() {
        leader.stopMotor();
    }

    /* ==================== COMMAND FACTORIES ==================== */

    public Command run(double speed) {
        return Commands.startEnd(
            () -> setSpeed(speed),
            this::stop,
            this
        );
    }

    public Command intake() {
        return run(Prefeed.INTAKE_SPEED);
    }

    public Command outtake() {
        return run(Prefeed.OUTTAKE_SPEED);
    }

    /* ==================== TELEMETRY ==================== */

    public double getCurrent() {
        return leader.getOutputCurrent();
    }

    public double getVelocity() {
        return leader.getEncoder().getVelocity();
    }

    public boolean isStalled() {
        return getCurrent() > Prefeed.STALL_CURRENT_THRESHOLD &&
               Math.abs(getVelocity()) < Prefeed.STALL_VELOCITY_THRESHOLD;
    }

    /* ==================== PERIODIC ==================== */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Prefeed/Current", getCurrent());
        SmartDashboard.putNumber("Prefeed/Velocity", getVelocity());
        SmartDashboard.putBoolean("Prefeed/Stalled", isStalled());
    }
}