package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.led.*;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led =
        new AddressableLED(Constants.LED.PWM_PORT);

    private final AddressableLEDBuffer buffer =
        new AddressableLEDBuffer(Constants.LED.LED_COUNT);

    private final LEDConfig config = new LEDConfig();

    private final LEDRenderer renderer =
        new LEDRenderer(buffer, config);

    private LEDState currentState =
        LEDState.IDLE;

    private LEDPattern manualPattern =
        LEDPatterns.rainbow();

    private LEDPattern activePattern =
        manualPattern;

    private boolean aprilTagTracking = false;

    private edu.wpi.first.wpilibj.util.Color currentColor =
        LEDColors.WHITE;

    private double animationTime = 0.0;

    private int lastCanTxErrors = 0;
    private int lastCanRxErrors = 0;
    private int lastCanBusOff = 0;

    private double lastCanFaultTimestamp = -1;

    public LEDSubsystem() {

        led.setLength(buffer.getLength());

        led.setData(buffer);

        led.start();
    }

    @Override
    public void periodic() {

        animationTime += 0.02 * config.animationSpeed;

        updateState();

        config.brightness =
            SmartDashboard.getNumber(
                "LED/Brightness",
                0.25
            );

        config.animationSpeed =
            SmartDashboard.getNumber(
                "LED/AnimationSpeed",
                1.0
            );

        activePattern.render(renderer, animationTime);

        led.setData(buffer);
    }

    private void updateState() {

        if (hasCanFault()) {

            currentState = LEDState.CAN_FAULT;

            activePattern =
                LEDPatterns.canFault();

            return;
        }

        if (!DriverStation.isDSAttached()) {

            currentState =
                LEDState.WAITING_FOR_RADIO;

            activePattern =
                LEDPatterns.breathing(
                    LEDColors.DODGER_BLUE,
                    3
                );

            return;
        }

        if (DriverStation.isDisabled()) {

            currentState =
                LEDState.DISABLED;

            activePattern =
                LEDPatterns.teamColors();

            return;
        }

        if (aprilTagTracking) {

            currentState =
                LEDState.APRILTAG_TRACKING;

            activePattern =
                LEDPatterns.chase(
                    LEDColors.ORANGE,
                    6,
                    20
                );

            return;
        }

        if (currentState == LEDState.MANUAL) {
            return;
        }

        currentState = LEDState.MANUAL;

        activePattern = manualPattern;
    }

    public void setState(
        LEDState state,
        LEDPattern pattern
    ) {

        currentState = state;

        activePattern = pattern;
    }

    public void setPattern(LEDPattern pattern) {

        activePattern = pattern;
    }

    public LEDConfig getConfig() {
        return config;
    }

    private boolean hasCanFault() {

        var s = RobotController.getCANStatus();

        boolean newFault =
            s.transmitErrorCount > lastCanTxErrors
            || s.receiveErrorCount > lastCanRxErrors
            || s.busOffCount > lastCanBusOff;

        lastCanTxErrors = s.transmitErrorCount;
        lastCanRxErrors = s.receiveErrorCount;
        lastCanBusOff = s.busOffCount;

        if (newFault) {
            lastCanFaultTimestamp =
                Timer.getFPGATimestamp();
        }

        return lastCanFaultTimestamp > 0
            && (
                Timer.getFPGATimestamp()
                - lastCanFaultTimestamp
            ) < 1.0;
    }

    public void setAprilTagTracking(boolean tracking) {

        aprilTagTracking = tracking;
    }

    public void setCustomColor(
        int r,
        int g,
        int b
    ) {

        currentColor =
            LEDColors.fromRGB(r, g, b);

        setSolidPattern();
    }

    public void setSolidPattern() {

        setPattern(
            LEDPatterns.solid(currentColor)
        );
    }

    public void setBreathingPattern() {

        setPattern(
            LEDPatterns.breathing(
                currentColor,
                3
            )
        );
    }

    public void setChasePattern() {

        setPattern(
            LEDPatterns.chase(
                currentColor,
                config.chaseSegmentLength,
                20
            )
        );
    }

    public void setKnightRiderPattern() {

        setPattern(
            LEDPatterns.knightRider(
                currentColor,
                20
            )
        );
    }

    public void setMeteorPattern() {

        setPattern(
            LEDPatterns.meteorRain(
                currentColor,
                config.meteorTrailLength,
                25
            )
        );
    }

    public void setRainbowPattern() {

        setPattern(
            LEDPatterns.rainbow()
        );
    }

    public Command holdPatternCommand(LEDPattern pattern) {

        return Commands.startEnd(
            () -> setPattern(pattern),
            () -> setPattern(LEDPatterns.rainbow()),
            this
        );
    }
}