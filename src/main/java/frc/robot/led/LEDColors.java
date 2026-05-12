package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDColors {

    private LEDColors() {}

    public static final Color RED = Color.kRed;
    public static final Color BLUE = Color.kBlue;
    public static final Color WHITE = Color.kWhite;
    public static final Color BLACK = Color.kBlack;
    public static final Color GREEN = Color.kGreen;
    public static final Color YELLOW = Color.kYellow;
    public static final Color ORANGE = Color.kOrange;
    public static final Color ORANGE_RED = Color.kOrangeRed;
    public static final Color PURPLE = Color.kPurple;
    public static final Color GOLD = Color.kGold;
    public static final Color HOT_PINK = Color.kHotPink;
    public static final Color DODGER_BLUE = Color.kDodgerBlue;
    public static final Color DEEP_SKY_BLUE = Color.kDeepSkyBlue;

    public static Color fromRGB(int r, int g, int b) {
        return new Color(
            clamp(r) / 255.0,
            clamp(g) / 255.0,
            clamp(b) / 255.0
        );
    } 

    public static Color scale(Color color, double brightness) {
        double scalar = Math.max(0.0, Math.min(1.0, brightness));

        return new Color(
            color.red * scalar,
            color.green * scalar,
            color.blue * scalar
        );
    }

    public static Color lerp(Color a, Color b, double t) {
        double clamped = Math.max(0.0, Math.min(1.0, t));

        return new Color(
            a.red + (b.red - a.red) * clamped,
            a.green + (b.green - a.green) * clamped,
            a.blue + (b.blue - a.blue) * clamped
        );
    }

    private static int clamp(int value) {
        return Math.max(0, Math.min(255, value));
    }
}