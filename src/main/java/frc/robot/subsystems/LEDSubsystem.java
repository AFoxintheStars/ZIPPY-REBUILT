package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.util.LEDAnimations;
import frc.robot.util.LEDColors;

public class LEDSubsystem extends SubsystemBase {
  public enum LEDMode { IDLE, CAN_FAULT, INTAKE_ACTIVE, PREFEED_ACTIVE, WAITING_FOR_RADIO, APRILTAG_TRACKING, DISABLED }

  public enum IdlePattern {
    RAINBOW, SOLID_RED, SOLID_WHITE, SOLID_BLUE, TEAM_COLORS, BREATHING_WHITE, CHASE,
    KNIGHT_RIDER, STROBE, THEATER_CHASE, SPARKLE, CUSTOM_RGB, FIREWORK, METEOR_RAIN,
    STACKING, OCEAN_WAVE, SUNRISE_SUNSET
  }

  private final AddressableLED led = new AddressableLED(Constants.LED.PWM_PORT);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED.LED_COUNT);
  private LEDMode currentMode = LEDMode.IDLE;
  private IdlePattern idlePattern = IdlePattern.RAINBOW;
  private double animationOffset = 0.0;
  private Color activeColor = Color.kWhite;
  private int lastCanTxErrors = 0;
  private int lastCanRxErrors = 0;
  private int lastCanBusOff = 0;
  private double lastCanFaultTimestamp = -1.0;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();

    SmartDashboard.putNumber("LED/CustomR", 255);
    SmartDashboard.putNumber("LED/CustomG", 255);
    SmartDashboard.putNumber("LED/CustomB", 255);
    SmartDashboard.putNumber("LED/Brightness", Constants.LED.BRIGHTNESS);
    SmartDashboard.putNumber("LED/AnimationSpeed", 1.0);
  }

  public void setMode(LEDMode mode) { currentMode = mode; }
  public Command setModeCommand(LEDMode mode) { return Commands.runOnce(() -> setMode(mode), this); }
  public Command holdModeCommand(LEDMode mode) { return Commands.startEnd(() -> setMode(mode), () -> setMode(LEDMode.IDLE), this); }
  public Command disabledCommand() { return Commands.run(() -> setMode(LEDMode.DISABLED), this); }
  public Command cycleIdlePatternCommand() { return Commands.runOnce(this::cycleIdlePattern, this); }
  public void cycleIdlePattern() { idlePattern = IdlePattern.values()[(idlePattern.ordinal() + 1) % IdlePattern.values().length]; }

  public void setIdlePattern(IdlePattern pattern) {
    switch (pattern) {
      case SOLID_RED -> activeColor = Color.kRed;
      case SOLID_WHITE -> activeColor = Color.kWhite;
      case SOLID_BLUE -> activeColor = Color.kBlue;
      default -> {}
    }
    idlePattern = pattern;
    currentMode = LEDMode.IDLE;
  }

  public void setCustomColor(int r, int g, int b) {
    activeColor = LEDColors.fromRGB(r, g, b);
    setIdlePattern(IdlePattern.CUSTOM_RGB);
  }

  private boolean hasCanFault() {
    var s = RobotController.getCANStatus();
    boolean newFault = s.transmitErrorCount > lastCanTxErrors || s.receiveErrorCount > lastCanRxErrors || s.busOffCount > lastCanBusOff;
    lastCanTxErrors = s.transmitErrorCount;
    lastCanRxErrors = s.receiveErrorCount;
    lastCanBusOff = s.busOffCount;
    if (newFault) lastCanFaultTimestamp = Timer.getFPGATimestamp();
    return lastCanFaultTimestamp > 0 && (Timer.getFPGATimestamp() - lastCanFaultTimestamp) < 1.0;
  }

  private boolean hasAllianceAprilTagTarget() {
    var latest = Cameras.TURRET_CAM.camera.getLatestResult();
    if (!latest.hasTargets()) return false;
    var alliance = DriverStation.getAlliance();
    int[] allowed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red ? Constants.VisionConstants.RED_HUB_TAGS : Constants.VisionConstants.BLUE_HUB_TAGS;
    for (var t : latest.getTargets()) for (int id : allowed) if (t.getFiducialId() == id) return true;
    return false;
  }

  @Override
  public void periodic() {
    double speed = Math.max(0.05, SmartDashboard.getNumber("LED/AnimationSpeed", 1.0));
    animationOffset += speed;

    if (idlePattern == IdlePattern.CUSTOM_RGB) {
      int r = (int) SmartDashboard.getNumber("LED/CustomR", 255);
      int g = (int) SmartDashboard.getNumber("LED/CustomG", 255);
      int b = (int) SmartDashboard.getNumber("LED/CustomB", 255);
      activeColor = LEDColors.fromRGB(r, g, b);
    }

    if (hasCanFault()) drawCanFault();
    else if (!DriverStation.isDSAttached()) drawWaitingForRadio();
    else if (hasAllianceAprilTagTarget()) drawAprilTagTracking();
    else switch (currentMode) {
      case INTAKE_ACTIVE -> LEDAnimations.chase(buffer, LEDColors.Palette.ORANGE_RED.color(), animationOffset, 6);
      case PREFEED_ACTIVE -> LEDAnimations.chase(buffer, LEDColors.Palette.PURPLE.color(), -animationOffset, 8);
      case APRILTAG_TRACKING -> drawAprilTagTracking();
      case DISABLED -> drawTeamColors();
      case CAN_FAULT -> drawCanFault();
      case WAITING_FOR_RADIO -> drawWaitingForRadio();
      case IDLE -> drawIdlePattern();
    }

    applyBrightness();
    led.setData(buffer);
  }

  private void drawIdlePattern() {
    switch (idlePattern) {
      case RAINBOW -> drawRainbow();
      case SOLID_RED, SOLID_WHITE, SOLID_BLUE, CUSTOM_RGB -> LEDAnimations.fill(buffer, activeColor);
      case TEAM_COLORS -> drawTeamColors();
      case BREATHING_WHITE -> LEDAnimations.fill(buffer, LEDColors.scale(activeColor, (Math.sin(Timer.getFPGATimestamp() * 2.5) + 1.0) * 0.5));
      case CHASE -> LEDAnimations.chase(buffer, activeColor, animationOffset, 6);
      case KNIGHT_RIDER -> drawKnightRider();
      case STROBE -> LEDAnimations.fill(buffer, ((int) (Timer.getFPGATimestamp() * 20) % 2) == 0 ? activeColor : Color.kBlack);
      case THEATER_CHASE -> drawTheaterChase();
      case SPARKLE -> drawSparkle();
      case FIREWORK -> LEDAnimations.firework(buffer, activeColor, animationOffset);
      case METEOR_RAIN -> LEDAnimations.meteorRain(buffer, activeColor, animationOffset);
      case STACKING -> LEDAnimations.stacking(buffer, activeColor, animationOffset);
      case OCEAN_WAVE -> LEDAnimations.oceanWave(buffer, Timer.getFPGATimestamp());
      case SUNRISE_SUNSET -> LEDAnimations.sunriseSunset(buffer, Timer.getFPGATimestamp());
    }
  }

  private void drawRainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, (int) ((i * 180.0 / buffer.getLength() + animationOffset) % 180), 255, 140);
    }
  }

  private void drawCanFault() { LEDAnimations.fill(buffer, ((int) (Timer.getFPGATimestamp() * 4) % 2) == 0 ? Color.kYellow : Color.kGreen); }
  private void drawWaitingForRadio() {
    double pulse = (Math.sin(Timer.getFPGATimestamp() * 3.0) + 1.0) * 0.5;
    for (int i = 0; i < buffer.getLength(); i++) buffer.setHSV(i, 120, 255, (int) (pulse * 180));
  }
  private void drawAprilTagTracking() { LEDAnimations.fill(buffer, ((int) (Timer.getFPGATimestamp() * 10) % 2) == 0 ? Color.kDeepSkyBlue : Color.kWhite); }
  private void drawTeamColors() { for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, (i % 2 == 0) ? Color.kRed : Color.kWhite); }

  private void drawKnightRider() {
    int max = Math.max(1, buffer.getLength() - 1);
    int period = max * 2;
    int frame = ((int) animationOffset) % period;
    int index = frame <= max ? frame : period - frame;
    LEDAnimations.fill(buffer, Color.kBlack);
    buffer.setLED(index, activeColor);
  }

  private void drawTheaterChase() {
    int shift = ((int) animationOffset) % 3;
    for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, ((i + shift) % 3 == 0) ? activeColor : Color.kBlack);
  }

  private void drawSparkle() {
    LEDAnimations.fill(buffer, Color.kBlack);
    for (int i = 0; i < Math.max(1, buffer.getLength() / 8); i++) buffer.setLED((int) (Math.random() * buffer.getLength()), activeColor);
  }

  private void applyBrightness() {
    double brightness = Math.max(0.0, Math.min(1.0, SmartDashboard.getNumber("LED/Brightness", Constants.LED.BRIGHTNESS)));
    for (int i = 0; i < buffer.getLength(); i++) {
      Color c = buffer.getLED(i);
      buffer.setLED(i, new Color(c.red * brightness, c.green * brightness, c.blue * brightness));
    }
  }
}
