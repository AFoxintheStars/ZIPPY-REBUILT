package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDRenderer {

    private final AddressableLEDBuffer buffer;

    private final LEDConfig config;

    public LEDRenderer(AddressableLEDBuffer buffer, LEDConfig config) {
        this.buffer = buffer;
        this.config = config;
    }

    public int length() {
        return buffer.getLength();
    }

    public void setLED(int index, Color color) {

        Color scaled = LEDColors.scale(color, config.brightness);

        buffer.setLED(index, scaled);
    }

    public void setHSV(int index, int h, int s, int v) {

        int scaledValue = (int)(v * config.brightness);

        buffer.setHSV(index, h, s, scaledValue);
    }

    public void fill(Color color) {
        for (int i = 0; i < length(); i++) {
            setLED(i, color);
        }
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }
}