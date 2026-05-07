package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDColors {
  public enum Palette {
    ORANGE_RED(Color.kOrangeRed),
    PURPLE(Color.kPurple),
    DEEP_SKY_BLUE(Color.kDeepSkyBlue),
    WHITE(Color.kWhite),
    RED(Color.kRed),
    BLUE(Color.kBlue),
    YELLOW(Color.kYellow),
    GREEN(Color.kGreen),
    DODGER_BLUE(Color.kDodgerBlue),
    GOLD(Color.kGold),
    ORANGE(Color.kOrange),
    HOT_PINK(Color.kHotPink);

    private final Color color;

    Palette(Color color) {
      this.color = color;
    }

    public Color color() {
      return color;
    }
  }

  private LEDColors() {}

  public static Color fromRGB(int r, int g, int b) {
    return new Color(clampChannel(r) / 255.0, clampChannel(g) / 255.0, clampChannel(b) / 255.0);
  }

  public static Color scale(Color base, double scalar) {
    double s = Math.max(0.0, Math.min(1.0, scalar));
    return new Color(base.red * s, base.green * s, base.blue * s);
  }

  public static int clampChannel(int value) {
    return Math.max(0, Math.min(255, value));
  }
}
