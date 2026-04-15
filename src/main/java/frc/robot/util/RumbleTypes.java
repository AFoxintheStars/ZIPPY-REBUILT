package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleTypes {

    /* ================= BASIC ================= */

    public static Command constant(CommandXboxController controller, double strength) {
        return Commands.startEnd(
            () -> set(controller, strength),
            () -> set(controller, 0)
        );
    }

    public static Command pulse(CommandXboxController controller, double strength, double duration) {
        return constant(controller, strength).withTimeout(duration);
    }

    /* ================= PATTERNS ================= */

    // Quick tap (button press feel)
    public static Command tap(CommandXboxController controller) {
        return pulse(controller, 0.4, 0.1);
    }

    // Double pulse (confirmation)
    public static Command doubleTap(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.5, 0.1),
            Commands.waitSeconds(0.1),
            pulse(controller, 0.5, 0.1)
        );
    }

    // Triple pulse (important event)
    public static Command tripleTap(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.6, 0.1),
            Commands.waitSeconds(0.1),
            pulse(controller, 0.6, 0.1),
            Commands.waitSeconds(0.1),
            pulse(controller, 0.6, 0.1)
        );
    }

    // Ramp up (spinning up shooter feel)
    public static Command rampUp(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.2, 0.2),
            pulse(controller, 0.4, 0.2),
            pulse(controller, 0.7, 0.3)
        );
    }

    // Warning (low/high limit, error)
    public static Command warning(CommandXboxController controller) {
        return Commands.repeatingSequence(
            pulse(controller, 0.8, 0.1),
            Commands.waitSeconds(0.1)
        ).withTimeout(0.6);
    }

    // Ready to shoot (distinct feel)
    public static Command ready(CommandXboxController controller) {
        return Commands.sequence(
            pulse(controller, 0.3, 0.08),
            Commands.waitSeconds(0.05),
            pulse(controller, 0.8, 0.15)
        );
    }

    // Continuous light rumble (while holding)
    public static Command softHold(CommandXboxController controller) {
        return constant(controller, 0.2);
    }

    // Strong hold (shooting / intake heavy load)
    public static Command strongHold(CommandXboxController controller) {
        return constant(controller, 0.6);
    }

    /* ================= INTERNAL ================= */

    private static void set(CommandXboxController controller, double value) {
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, value);
    }
}