package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDPatterns {

    private LEDPatterns() {}

    public static LEDPattern solid(Color color) {

        return (renderer, time) -> renderer.fill(color);
    }

    public static LEDPattern rainbow() {

        return (renderer, time) -> {

            for (int i = 0; i < renderer.length(); i++) {

                renderer.setHSV(
                    i,
                    (int)((i * 180.0 / renderer.length() + time * 25) % 180),
                    255,
                    140
                );
            }
        };
    }

    public static LEDPattern strobe(Color color, double speed) {

        return (renderer, time) -> {

            boolean on = ((int)(time * speed)) % 2 == 0;

            renderer.fill(on ? color : LEDColors.BLACK);
        };
    }

    public static LEDPattern breathing(Color color, double speed) {

        return (renderer, time) -> {

            double pulse =
                (Math.sin(time * speed) + 1.0) * 0.5;

            renderer.fill(LEDColors.scale(color, pulse));
        };
    }

    public static LEDPattern chase(
        Color color,
        int segmentLength,
        double speed
    ) {

        return (renderer, time) -> {

            int offset = (int)(time * speed);

            for (int i = 0; i < renderer.length(); i++) {

                int phase =
                    (i - offset) % segmentLength;

                if (phase < 0) {
                    phase += segmentLength;
                }

                renderer.setLED(
                    i,
                    phase < segmentLength / 2
                        ? color
                        : LEDColors.BLACK
                );
            }
        };
    }

    public static LEDPattern knightRider(
        Color color,
        double speed
    ) {

        return (renderer, time) -> {

            renderer.fill(LEDColors.BLACK);

            int max = renderer.length() - 1;

            int period = max * 2;

            int frame =
                ((int)(time * speed)) % period;

            int index =
                frame <= max
                    ? frame
                    : period - frame;

            renderer.setLED(index, color);
        };
    }

    public static LEDPattern sparkle(Color color) {

        return (renderer, time) -> {

            renderer.fill(LEDColors.BLACK);

            for (int i = 0; i < renderer.length() / 8; i++) {

                renderer.setLED(
                    (int)(Math.random() * renderer.length()),
                    color
                );
            }
        };
    }

    public static LEDPattern meteorRain(
        Color color,
        int trailLength,
        double speed
    ) {

        return (renderer, time) -> {

            renderer.fill(LEDColors.BLACK);

            int head =
                ((int)(time * speed)) % renderer.length();

            for (int t = 0; t < trailLength; t++) {

                int index = head - t;

                if (index < 0) {
                    index += renderer.length();
                }

                double intensity =
                    1.0 - (t / (double)trailLength);

                renderer.setLED(
                    index,
                    LEDColors.scale(
                        color,
                        intensity * intensity
                    )
                );
            }
        };
    }

    public static LEDPattern teamColors() {

            return (renderer, time) -> {

                for (int i = 0; i < renderer.length(); i++) {

                    renderer.setLED(
                        i,
                        i % 2 == 0
                            ? LEDColors.RED
                            : LEDColors.WHITE
                    );
                }
            };
        }

    public static LEDPattern canFault() {

    return (renderer, time) -> {

        boolean flip =
            ((int)(time * 8)) % 2 == 0;

        for (int i = 0; i < renderer.length(); i++) {

            renderer.setLED(
                i,
                (i % 2 == 0) == flip
                    ? LEDColors.YELLOW
                    : LEDColors.GREEN
            );
        }
    };
}

    public static LEDPattern oceanWave() {

        return (renderer, time) -> {

            for (int i = 0; i < renderer.length(); i++) {

                double wave =
                    (Math.sin(i * 0.18 + time * 1.7)
                    + Math.sin(i * 0.09 + time * 1.1))
                    * 0.25 + 0.5;

                renderer.setLED(
                    i,
                    LEDColors.fromRGB(
                        15,
                        (int)(30 + 80 * wave),
                        (int)(120 + 100 * wave)
                    )
                );
            }
        };
    }
}