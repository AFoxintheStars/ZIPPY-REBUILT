package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  public enum LEDMode {
    RAINBOW,
    CAN_FAULT,
    INTAKE_ACTIVE,
    PREFEED_ACTIVE,
    WAITING_FOR_RADIO,
    APRILTAG_TRACKING,
    IDLE_WAVE,
    SHOOT_READY,
    CLIMB_WARNING
  }

  private final AddressableLED led = new AddressableLED(Constants.LED.PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.LED_COUNT);

  private LEDMode currentMode = LEDMode.RAINBOW;
  private double animationOffset = 0.0;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void setMode(LEDMode mode) {
    currentMode = mode;
  }

  public Command setModeCommand(LEDMode mode) {
    return Commands.runOnce(() -> setMode(mode), this);
  }

  public Command holdModeCommand(LEDMode mode) {
    return Commands.startEnd(() -> setMode(mode), () -> setMode(LEDMode.RAINBOW), this);
  }

  private boolean hasCanFault() {
    var canStatus = RobotController.getCANStatus();
    return canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0 || canStatus.busOffCount > 0;
  }

  private void setScaledHSV(int i, int h, int s, int v) {
    int scaledV = (int) (Math.max(0.0, Math.min(1.0, Constants.LED.BRIGHTNESS)) * v);
    buffer.setHSV(i, h, s, scaledV);
  }

  @Override
  public void periodic() {
    animationOffset += 1.0;

    if (hasCanFault()) {
      drawCanFault();
    } else if (!DriverStation.isDSAttached()) {
      drawWaitingForRadio();
    } else {
      switch (currentMode) {
        case INTAKE_ACTIVE -> drawIntakeActive();
        case PREFEED_ACTIVE -> drawPrefeedActive();
        case APRILTAG_TRACKING -> drawAprilTagTracking();
        case IDLE_WAVE -> drawIdleWave();
        case SHOOT_READY -> drawShootReady();
        case CLIMB_WARNING -> drawClimbWarning();
        case CAN_FAULT -> drawCanFault();
        case WAITING_FOR_RADIO -> drawWaitingForRadio();
        case RAINBOW -> drawRainbow();
      }
    }

    led.setData(buffer);
  }

  private void drawRainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      int hue = (int) ((i * 180.0 / buffer.getLength() + animationOffset) % 180);
      setScaledHSV(i, hue, 255, 140);
    }
  }

  private void drawCanFault() {
    boolean flash = ((int) (Timer.getFPGATimestamp() * 8) % 2) == 0;
    Color a = flash ? Color.kYellow : Color.kGreen;
    Color b = flash ? Color.kGreen : Color.kYellow;
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, (i % 2 == 0) ? a : b);
    }
  }

  private void drawIntakeActive() {
    for (int i = 0; i < buffer.getLength(); i++) {
      boolean on = ((i + (int) animationOffset) % 6) < 3;
      buffer.setLED(i, on ? Color.kOrangeRed : Color.kBlack);
    }
  }

  private void drawPrefeedActive() {
    for (int i = 0; i < buffer.getLength(); i++) {
      boolean on = ((i - (int) animationOffset) % 8 + 8) % 8 < 2;
      buffer.setLED(i, on ? Color.kPurple : Color.kBlack);
    }
  }

  private void drawWaitingForRadio() {
    double pulse = (Math.sin(Timer.getFPGATimestamp() * 3.0) + 1.0) * 0.5;
    int value = (int) (pulse * 180);
    for (int i = 0; i < buffer.getLength(); i++) {
      setScaledHSV(i, 120, 255, value);
    }
  }

  private void drawIdleWave() {
    for (int i = 0; i < buffer.getLength(); i++) {
      double wave = (Math.sin((i * 0.4) + (animationOffset * 0.15)) + 1.0) * 0.5;
      setScaledHSV(i, 20, 200, (int) (wave * 120));
    }
  }

  private void drawAprilTagTracking() {
    for (int i = 0; i < buffer.getLength(); i++) {
      boolean trackingPixel = ((i + (int) animationOffset) % 10) < 4;
      buffer.setLED(i, trackingPixel ? Color.kWhite : Color.kDeepSkyBlue);
    }
  }

  private void drawShootReady() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, Color.kLimeGreen);
    }
  }

  private void drawClimbWarning() {
    boolean flash = ((int) (Timer.getFPGATimestamp() * 12) % 2) == 0;
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, flash ? Color.kRed : Color.kBlack);
    }
  }
}
