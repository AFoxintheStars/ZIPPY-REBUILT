package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {

    /* ==================== HARDWARE ==================== */

    private final Servo hoodServo =
        new Servo(HoodConstants.SERVO_PORT);

    private final DutyCycleEncoder hoodEncoder =
        new DutyCycleEncoder(HoodConstants.ENCODER_PORT);

    /* ==================== STATE ==================== */

    private double currentSpeed = 0;

    /* ===================== CONSTRUCTOR ==================== */

    public HoodSubsystem() {
        hoodEncoder.setDutyCycleRange(
            HoodConstants.DUTY_MIN,
            HoodConstants.DUTY_MAX
        );
    }

    /* ==================== CONTROL ==================== */

    public void setSpeed(double speed) {
        speed = Math.max(-1.0, Math.min(1.0, speed));

        if (Math.abs(speed) < 0.05) speed = 0;

        if (!isEncoderConnected()) {
            stop();
            return; 
        }

        double angle = getAngle();

        if (speed > 0 && angle >= HoodConstants.MAX_ANGLE) speed = 0;
        if (speed < 0 && angle <= HoodConstants.MIN_ANGLE) speed = 0;

        currentSpeed = speed;

        hoodServo.set(0.5 + (speed * 0.5));
    }

    public void stop() {
        hoodServo.set(0.5);
        currentSpeed = 0;
    }

    /* ==================== SENSORS ==================== */

    public double getAngle() {
        double rotations = hoodEncoder.get();

        double angle = rotations * 360.0 * (48.0 / 18.0);

        angle -= HoodConstants.ZERO_OFFSET;

        return angle;   
    }

    public boolean isEncoderConnected() {
        return hoodEncoder.isConnected();
    }

    /* ==================== MANUAL COMMANDS ==================== */

    public Command moveDown() {
        return Commands.startEnd(
            () -> setSpeed(HoodConstants.DOWN_SPEED),
            this::stop,
            this
        );
    }

    public Command moveServoUp() {
        return Commands.startEnd(
            () -> hoodServo.set(HoodConstants.UP_SPEED),
            this::stop,
            this
        );
    }

    public Command moveServoDown() {
        return Commands.startEnd(
            () -> hoodServo.set(HoodConstants.DOWN_SPEED),
            this::stop,
            this
        );
    }

    public Command moveUp() { 
        return Commands.startEnd(
            () -> setSpeed(HoodConstants.UP_SPEED),
            this::stop,
            this
        );
    }

    /* ==================== TELEMETRY ==================== */

    @Override
    public void periodic() {
        double angle = getAngle();

        SmartDashboard.putNumber("Hood/Angle", angle);
        SmartDashboard.putNumber("Hood/Raw Angle", hoodEncoder.get() * (48.0 / 18.0) * 360.0);
        SmartDashboard.putBoolean("Hood/Connected", isEncoderConnected());

        SmartDashboard.putBoolean("Hood/Upper Limit", angle >= HoodConstants.MAX_ANGLE);
        SmartDashboard.putBoolean("Hood/Lower Limit", angle <= HoodConstants.MIN_ANGLE);

        SmartDashboard.putNumber("Hood/Speed", currentSpeed);
    }
}