package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants.HoodConstants;
import frc.robot.util.ShooterLookupTable;
import frc.robot.util.ShotData;
import java.util.NavigableMap;

public class HoodSubsystem extends SubsystemBase {

    /* ==================== HARDWARE ==================== */

    private final Servo hoodServo =
        new Servo(HoodConstants.SERVO_PORT);

    private final DutyCycleEncoder hoodEncoder =
        new DutyCycleEncoder(HoodConstants.ENCODER_PORT);

    /* ==================== STATE ==================== */

    private double currentSpeed = 0;
    private double currentServoSpeed = 0;
    private double targetAngle = Double.NaN;
    private final NavigableMap<Double, ShotData> shooterLookupTable =
        ShooterLookupTable.loadFromDeployCSV("shooter_lookup_table.csv");
    private double zeroOffsetDeg = HoodConstants.ZERO_OFFSET;
    private boolean angleTrackerInitialized = false;
    private double lastRawRotations = 0.0;
    private double continuousRotations = 0.0;

    /* ===================== CONSTRUCTOR ==================== */

    public HoodSubsystem() {
        hoodEncoder.setDutyCycleRange(
            HoodConstants.DUTY_MIN,
            HoodConstants.DUTY_MAX
        );

        SmartDashboard.putData("Hood/ZeroToCurrent",
            Commands.runOnce(this::zeroAngleToCurrentPosition, this));
    }

    /* ==================== CONTROL ==================== */

    public void setSpeed(double speed) {
        // speed is in angle space: +speed should increase hood angle.
        double limitedSpeed = Math.max(-1.0, Math.min(1.0, speed));

        if (Math.abs(limitedSpeed) < 0.05) limitedSpeed = 0;

        if (isEncoderConnected()) {
            double angle = getAngle();
            if (limitedSpeed > 0 && angle >= HoodConstants.MAX_ANGLE) limitedSpeed = 0;
            if (limitedSpeed < 0 && angle <= HoodConstants.MIN_ANGLE) limitedSpeed = 0;
        }

        currentSpeed = limitedSpeed;
        currentServoSpeed = HoodConstants.SERVO_INVERTED ? -limitedSpeed : limitedSpeed;

        hoodServo.set(0.5 + (currentServoSpeed * 0.5));
    }

    public void stop() {
        hoodServo.set(0.5);
        currentSpeed = 0;
        currentServoSpeed = 0;
    }

    public void clearTargetAngle() {
        targetAngle = Double.NaN;
    }

    public void setTargetAngle(double angleDeg) {
        targetAngle = clampAngle(angleDeg);
        runClosedLoop();
    }

    public void setTargetAngleFromDistance(double distanceMeters) {
        setTargetAngle(getLookupAngle(distanceMeters));
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getLookupAngle(double distanceMeters) {
        if (shooterLookupTable.isEmpty()) {
            return interpolate(distanceMeters, HoodConstants.HOOD_LOOKUP);
        }
        return ShooterLookupTable.interpolate(distanceMeters, shooterLookupTable).angleDeg;
    }

    public void zeroToCurrent() {
        zeroOffsetDeg = getContinuousRawAngleDeg();
    }

    private void runClosedLoop() {
        if (!isEncoderConnected() || Double.isNaN(targetAngle)) {
            stop();
            return;
        }

        double error = targetAngle - getAngle();
        if (Math.abs(error) <= HoodConstants.ANGLE_TOLERANCE_DEG) {
            stop();
            return;
        }

        double speedCmd = (error * HoodConstants.TRACKING_KP)
            + Math.copySign(HoodConstants.TRACKING_KS, error);
        speedCmd = Math.max(-HoodConstants.TRACKING_MAX_SPEED,
            Math.min(HoodConstants.TRACKING_MAX_SPEED, speedCmd));

        setSpeed(speedCmd);
    }

    /* ==================== SENSORS ==================== */

    public double getAngle() {
        double rawAngle = getContinuousRawAngleDeg();
        double direction = HoodConstants.ENCODER_INVERTED ? -1.0 : 1.0;
        double angle = (rawAngle - zeroOffsetDeg) * direction;
        return angle;
    }

    public boolean isEncoderConnected() {
        return hoodEncoder.isConnected();
    }

    public void zeroAngleToCurrentPosition() {
        zeroOffsetDeg = getContinuousRawAngleDeg();
    }

    public void setZeroOffsetDegrees(double zeroOffsetDeg) {
        this.zeroOffsetDeg = zeroOffsetDeg;
    }

    public double getZeroOffsetDegrees() {
        return zeroOffsetDeg;
    }

    /* ==================== MANUAL COMMANDS ==================== */

    public Command moveDown() {
        return Commands.startEnd(
            () -> hoodServo.set(HoodConstants.DOWN_SPEED),
            this::stop,
            this
        );
    }

    public Command moveServoUp() {
        return Commands.startEnd(
            () -> setSpeed(HoodConstants.UP_SPEED),
            this::stop,
            this
        );
    }

    public Command moveServoDown() {
        return Commands.startEnd(
            () -> setSpeed(HoodConstants.DOWN_SPEED),
            this::stop,
            this
        );
    }

    public Command moveUp() { 
        return Commands.startEnd(
            () -> hoodServo.set(HoodConstants.UP_SPEED),
            this::stop,
            this
        );
    }

    /* ==================== TELEMETRY ==================== */

    @Override
    public void periodic() {
        double angle = getAngle();
        double error = Double.isNaN(targetAngle) ? 0.0 : (targetAngle - angle);

        SmartDashboard.putNumber("Hood/Angle", angle);
        SmartDashboard.putNumber("Hood/Raw Angle", hoodEncoder.get() * (48.0 / 18.0) * 360.0);
        SmartDashboard.putBoolean("Hood/EncoderInverted", HoodConstants.ENCODER_INVERTED);
        SmartDashboard.putBoolean("Hood/ServoInverted", HoodConstants.SERVO_INVERTED);
        SmartDashboard.putBoolean("Hood/Connected", isEncoderConnected());
        SmartDashboard.putNumber("Hood/ZeroOffsetDeg", zeroOffsetDeg);
        SmartDashboard.putNumber("Hood/LookupRows", shooterLookupTable.size());

        SmartDashboard.putBoolean("Hood/Upper Limit", angle >= HoodConstants.MAX_ANGLE);
        SmartDashboard.putBoolean("Hood/Lower Limit", angle <= HoodConstants.MIN_ANGLE);

        SmartDashboard.putNumber("Hood/Speed", currentSpeed);
        SmartDashboard.putNumber("Hood/ServoSpeed", currentServoSpeed);
        SmartDashboard.putNumber("Hood/TargetAngle", targetAngle);
        SmartDashboard.putNumber("Hood/ErrorDeg", error);
    }

    private double getContinuousRawAngleDeg() {
        double rawRotations = hoodEncoder.get();
        if (!angleTrackerInitialized) {
            angleTrackerInitialized = true;
            lastRawRotations = rawRotations;
            continuousRotations = rawRotations;
        } else {
            double delta = rawRotations - lastRawRotations;
            if (delta > 0.5) {
                delta -= 1.0;
            } else if (delta < -0.5) {
                delta += 1.0;
            }
            continuousRotations += delta;
            lastRawRotations = rawRotations;
        }

        return continuousRotations * 360.0 * (48.0 / 18.0);
    }

    private static double clampAngle(double angleDeg) {
        return Math.max(HoodConstants.MIN_ANGLE, Math.min(HoodConstants.MAX_ANGLE, angleDeg));
    }

    private static double interpolate(double x, double[][] table) {
        if (table.length == 0) {
            return 0.0;
        }

        if (x <= table[0][0]) {
            return table[0][1];
        }

        if (x >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }

        for (int i = 0; i < table.length - 1; i++) {
            double x1 = table[i][0];
            double y1 = table[i][1];
            double x2 = table[i + 1][0];
            double y2 = table[i + 1][1];

            if (x >= x1 && x <= x2) {
                double t = (x - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }

        return table[table.length - 1][1];
    }
}
