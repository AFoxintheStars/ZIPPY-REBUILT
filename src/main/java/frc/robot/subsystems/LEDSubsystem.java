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
    IDLE,
    CAN_FAULT,
    INTAKE_ACTIVE,
    PREFEED_ACTIVE,
    WAITING_FOR_RADIO,
    APRILTAG_TRACKING,
    DISABLED
  }

  public enum IdlePattern {
    RAINBOW,
    SOLID_RED,
    SOLID_WHITE,
    SOLID_BLUE,
    TEAM_COLORS,
    BREATHING_WHITE
  }

  private final AddressableLED led = new AddressableLED(Constants.LED.PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.LED_COUNT);

  private LEDMode currentMode = LEDMode.IDLE;
  private IdlePattern idlePattern = IdlePattern.RAINBOW;
  private double animationOffset = 0.0;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void setMode(LEDMode mode) { currentMode = mode; }
  public void cycleIdlePattern() {
    IdlePattern[] patterns = IdlePattern.values();
    idlePattern = patterns[(idlePattern.ordinal() + 1) % patterns.length];
  }

  public void cycleIdleColorPattern() {
    switch (idlePattern) {
      case RAINBOW -> idlePattern = IdlePattern.SOLID_RED;
      case SOLID_RED -> idlePattern = IdlePattern.SOLID_WHITE;
      case SOLID_WHITE -> idlePattern = IdlePattern.SOLID_BLUE;
      case SOLID_BLUE -> idlePattern = IdlePattern.TEAM_COLORS;
      case TEAM_COLORS -> idlePattern = IdlePattern.BREATHING_WHITE;
      case BREATHING_WHITE -> idlePattern = IdlePattern.RAINBOW;
    }
  }

  public Command setModeCommand(LEDMode mode) {
    return Commands.runOnce(() -> setMode(mode), this);
  }

  public Command holdModeCommand(LEDMode mode) {
    return Commands.startEnd(() -> setMode(mode), () -> setMode(LEDMode.IDLE), this);
  }

  public Command disabledCommand() {
    return Commands.run(() -> setMode(LEDMode.DISABLED), this);
  }

  public Command cycleIdlePatternCommand() {
    return Commands.runOnce(this::cycleIdlePattern, this);
  }

  public Command cycleIdleColorPatternCommand() {
    return Commands.runOnce(this::cycleIdleColorPattern, this);
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
        case DISABLED -> drawDisabled();
        case CAN_FAULT -> drawCanFault();
        case WAITING_FOR_RADIO -> drawWaitingForRadio();
        case IDLE -> drawIdlePattern();
      }
    }

    led.setData(buffer);
  }

  private void drawIdlePattern() {
    switch (idlePattern) {
      case RAINBOW -> drawRainbow();
      case SOLID_RED -> fillColor(Color.kRed);
      case SOLID_WHITE -> fillColor(Color.kWhite);
      case SOLID_BLUE -> fillColor(Color.kBlue);
      case TEAM_COLORS -> drawTeamColors();
      case BREATHING_WHITE -> drawBreathingWhite();
    }
  }

  private void drawRainbow() { for (int i=0;i<buffer.getLength();i++) setScaledHSV(i,(int)((i*180.0/buffer.getLength()+animationOffset)%180),255,140); }

  private void drawCanFault() {
    boolean yellowFrame = ((int) (Timer.getFPGATimestamp() * 4) % 2) == 0;
    fillColor(yellowFrame ? Color.kYellow : Color.kGreen);
  }

  private void drawIntakeActive() { for (int i=0;i<buffer.getLength();i++) buffer.setLED(i,((i+(int)animationOffset)%6)<3?Color.kOrangeRed:Color.kBlack); }
  private void drawPrefeedActive() { for (int i=0;i<buffer.getLength();i++) buffer.setLED(i,((i-(int)animationOffset)%8+8)%8<2?Color.kPurple:Color.kBlack); }

  private void drawWaitingForRadio() {
    double pulse = (Math.sin(Timer.getFPGATimestamp() * 3.0) + 1.0) * 0.5;
    for (int i = 0; i < buffer.getLength(); i++) setScaledHSV(i, 120, 255, (int) (pulse * 180));
  }

  private void drawAprilTagTracking() {
    boolean flash = ((int) (Timer.getFPGATimestamp() * 10) % 2) == 0;
    fillColor(flash ? Color.kDeepSkyBlue : Color.kWhite);
  }

  private void drawTeamColors() {
    for (int i = 0; i < buffer.getLength(); i++) {
      int section = (i * 3) / Math.max(1, buffer.getLength());
      buffer.setLED(i, section == 0 ? Color.kRed : section == 1 ? Color.kWhite : Color.kBlack);
    }
  }

  private void drawBreathingWhite() {
    double pulse = (Math.sin(Timer.getFPGATimestamp() * 2.5) + 1.0) * 0.5;
    int v = (int) (pulse * 200);
    for (int i = 0; i < buffer.getLength(); i++) setScaledHSV(i, 0, 0, v);
  }

  private void drawDisabled() { drawTeamColors(); }

  private void fillColor(Color c) { for (int i=0;i<buffer.getLength();i++) buffer.setLED(i,c); }
}
