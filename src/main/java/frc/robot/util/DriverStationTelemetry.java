package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import java.util.OptionalInt;

public class DriverStationTelemetry
{
    public static void update()
    {
    // =========================
    // Game Data
    // =========================
    String gameData = DriverStation.getGameSpecificMessage();

    SmartDashboard.putString("DS/GameData/Raw", gameData);

    // Default values
    boolean validData = false;
    boolean redInactiveFirst = false;

    if (!gameData.isEmpty())
    {
        char c = gameData.charAt(0);

        if (c == 'R')
        {
            validData = true;
            redInactiveFirst = true;
            SmartDashboard.putString("DS/GameData/Result", "Red inactive first");
        }
        else if (c == 'B')
        {
            validData = true;
            redInactiveFirst = false;
            SmartDashboard.putString("DS/GameData/Result", "Blue inactive first");
        }
        else
        {
            SmartDashboard.putString("DS/GameData/Result", "INVALID DATA");
        }
    }
    else
    {
        SmartDashboard.putString("DS/GameData/Result", "No Data Yet");
    }

    SmartDashboard.putBoolean("DS/GameData/Valid", validData);
    SmartDashboard.putBoolean("DS/GameData/RedInactiveFirst", redInactiveFirst);

    // =========================
    // Hub State
    // =========================
    SmartDashboard.putBoolean("Hub Active", HubTracker.isActive());
    SmartDashboard.putString("Current Shift",
    HubTracker.getCurrentShift().map(Enum::name).orElse("None"));

    // =========================
    // Robot State
    // =========================
    SmartDashboard.putBoolean("DS/Enabled", DriverStation.isEnabled());
    SmartDashboard.putBoolean("DS/Disabled", DriverStation.isDisabled());
    SmartDashboard.putBoolean("DS/EStopped", DriverStation.isEStopped());

    SmartDashboard.putBoolean("DS/Autonomous", DriverStation.isAutonomous());
    SmartDashboard.putBoolean("DS/Teleop", DriverStation.isTeleop());
    SmartDashboard.putBoolean("DS/Test", DriverStation.isTest());

    SmartDashboard.putBoolean("DS/Autonomous Enabled", DriverStation.isAutonomousEnabled());
    SmartDashboard.putBoolean("DS/Teleop Enabled", DriverStation.isTeleopEnabled());
    SmartDashboard.putBoolean("DS/Test Enabled", DriverStation.isTestEnabled());

    SmartDashboard.putBoolean("DS/DS Attached", DriverStation.isDSAttached());
    SmartDashboard.putBoolean("DS/FMS Attached", DriverStation.isFMSAttached());

    // =========================
    // Match Info
    // =========================
    SmartDashboard.putString("DS/Event Name", DriverStation.getEventName());
    SmartDashboard.putNumber("DS/Match Number", DriverStation.getMatchNumber());
    SmartDashboard.putNumber("DS/Replay Number", DriverStation.getReplayNumber());
    SmartDashboard.putString("DS/Match Type", DriverStation.getMatchType().toString());

    SmartDashboard.putString("DS/Game Message", DriverStation.getGameSpecificMessage());
    SmartDashboard.putNumber("DS/Match Time", DriverStation.getMatchTime());

    // =========================
    // Alliance Info
    // =========================
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    SmartDashboard.putString("DS/Alliance", alliance.map(Enum::toString).orElse("Unknown"));

    OptionalInt location = DriverStation.getLocation();
    SmartDashboard.putNumber("DS/Alliance Station", location.orElse(-1));

    SmartDashboard.putString("DS/Raw Alliance Station", 
        DriverStation.getRawAllianceStation().toString());

    // =========================
    // Joystick Info
    // =========================
    for (int i = 0; i < DriverStation.kJoystickPorts; i++)
    {
        String base = "DS/Joystick" + i + "/";

        boolean connected = DriverStation.isJoystickConnected(i);
        SmartDashboard.putBoolean(base + "Connected", connected);

        if (!connected) continue;

        SmartDashboard.putString(base + "Name", DriverStation.getJoystickName(i));
        SmartDashboard.putBoolean(base + "Is Xbox", DriverStation.getJoystickIsXbox(i));
        SmartDashboard.putNumber(base + "Type", DriverStation.getJoystickType(i));

        int axisCount = DriverStation.getStickAxisCount(i);
        int buttonCount = DriverStation.getStickButtonCount(i);
        int povCount = DriverStation.getStickPOVCount(i);

        SmartDashboard.putNumber(base + "Axis Count", axisCount);
        SmartDashboard.putNumber(base + "Button Count", buttonCount);
        SmartDashboard.putNumber(base + "POV Count", povCount);

        // Axes
        for (int a = 0; a < axisCount; a++)
        {
            SmartDashboard.putNumber(base + "Axis " + a, 
                DriverStation.getStickAxis(i, a));
        }

        // Buttons
        for (int b = 1; b <= buttonCount; b++)
        {
            SmartDashboard.putBoolean(base + "Button " + b, 
                DriverStation.getStickButton(i, b));
        }

        // POVs
        for (int p = 0; p < povCount; p++)
        {
            SmartDashboard.putNumber(base + "POV " + p, 
                DriverStation.getStickPOV(i, p));
        }
    }
}
}