package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDAnimations {
  private LEDAnimations() {}


  public static void rainbow(AddressableLEDBuffer buffer, double offset) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, (int) ((i * 180.0 / buffer.getLength() + offset) % 180), 255, 140);
    }
  }

  public static void knightRider(AddressableLEDBuffer buffer, Color color, double offset) {
    int max = Math.max(1, buffer.getLength() - 1);
    int period = max * 2;
    int frame = ((int) offset) % period;
    int index = frame <= max ? frame : period - frame;
    fill(buffer, Color.kBlack);
    buffer.setLED(index, color);
  }

  public static void sparkle(AddressableLEDBuffer buffer, Color color) {
    fill(buffer, Color.kBlack);
    for (int i = 0; i < Math.max(1, buffer.getLength() / 8); i++) {
      buffer.setLED((int) (Math.random() * buffer.getLength()), color);
    }
  }

  public static void fill(AddressableLEDBuffer buffer, Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  public static void chase(AddressableLEDBuffer buffer, Color color, double offset, int segment) {
    for (int i = 0; i < buffer.getLength(); i++) {
      int phase = (i - (int) offset) % segment;
      if (phase < 0) phase += segment;
      buffer.setLED(i, phase < segment / 2 ? color : Color.kBlack);
    }
  }

  public static void meteorRain(AddressableLEDBuffer buffer, Color color, double offset) {
    fill(buffer, Color.kBlack);
    int head = ((int) offset) % buffer.getLength();
    int trail = Math.max(6, buffer.getLength() / 10);
    for (int t = 0; t < trail; t++) {
      int idx = head - t;
      if (idx < 0) idx += buffer.getLength();
      double intensity = 1.0 - (t / (double) trail);
      buffer.setLED(idx, LEDColors.scale(color, intensity * intensity));
    }
  }

  public static void firework(AddressableLEDBuffer buffer, Color base, double offset) {
    fill(buffer, LEDColors.scale(base, 0.08));
    int center = ((int) (offset * 0.5)) % buffer.getLength();
    int radius = ((int) offset / 8) % Math.max(2, buffer.getLength() / 4);
    for (int i = 0; i < buffer.getLength(); i++) {
      int d = Math.abs(i - center);
      if (d == radius || d == radius + 1) {
        buffer.setLED(i, LEDColors.Palette.GOLD.color());
      }
    }
  }

  public static void stacking(AddressableLEDBuffer buffer, Color color, double offset) {
    fill(buffer, Color.kBlack);
    int length = buffer.getLength();
    int stacked = ((int) (offset / 6)) % (length + 1);
    for (int i = length - 1; i >= length - stacked; i--) {
      if (i >= 0) buffer.setLED(i, color);
    }
    int flyer = length - 1 - (int) (offset % Math.max(1, (length - stacked)));
    if (flyer >= 0 && flyer < length - stacked) {
      buffer.setLED(flyer, LEDColors.Palette.WHITE.color());
    }
  }

  public static void oceanWave(AddressableLEDBuffer buffer, double time) {
    for (int i = 0; i < buffer.getLength(); i++) {
      double wave = (Math.sin(i * 0.18 + time * 1.7) + Math.sin(i * 0.09 + time * 1.1)) * 0.25 + 0.5;
      int blue = (int) (120 + 100 * wave);
      int green = (int) (30 + 80 * wave);
      buffer.setRGB(i, 15, green, blue);
    }
  }

  public static void sunriseSunset(AddressableLEDBuffer buffer, double time) {
    double phase = (Math.sin(time * 0.35) + 1.0) * 0.5;
    Color dawn = LEDColors.fromRGB(255, 80, 20);
    Color noon = LEDColors.fromRGB(255, 235, 150);
    Color dusk = LEDColors.fromRGB(180, 80, 220);
    Color blendA = lerp(dawn, noon, phase);
    Color blendB = lerp(noon, dusk, phase);
    for (int i = 0; i < buffer.getLength(); i++) {
      double mix = i / (double) Math.max(1, buffer.getLength() - 1);
      buffer.setLED(i, lerp(blendA, blendB, mix));
    }
  }

  public static Color lerp(Color a, Color b, double t) {
    double c = Math.max(0.0, Math.min(1.0, t));
    return new Color(a.red + (b.red - a.red) * c, a.green + (b.green - a.green) * c,
        a.blue + (b.blue - a.blue) * c);
  }
}
