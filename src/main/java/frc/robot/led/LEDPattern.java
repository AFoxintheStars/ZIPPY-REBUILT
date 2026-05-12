package frc.robot.led;

@FunctionalInterface
public interface LEDPattern {

    void render(
        LEDRenderer renderer,
        double time
    );
}