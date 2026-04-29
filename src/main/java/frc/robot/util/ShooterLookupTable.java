package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public final class ShooterLookupTable {
    private ShooterLookupTable() {}

    public static NavigableMap<Double, ShotData> loadFromDeployCSV(String filename) {
        NavigableMap<Double, ShotData> table = new TreeMap<>();
        File file = new File(Filesystem.getDeployDirectory(), filename);

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine(); // header
            if (line == null) {
                DriverStation.reportWarning("Shooter lookup table is empty: " + file.getAbsolutePath(), false);
                return table;
            }

            while ((line = reader.readLine()) != null) {
                String trimmed = line.trim();
                if (trimmed.isEmpty() || trimmed.startsWith("#")) {
                    continue;
                }

                String[] parts = trimmed.split(",");
                if (parts.length < 3) {
                    DriverStation.reportWarning("Skipping bad shooter table row: " + trimmed, false);
                    continue;
                }

                double distance = Double.parseDouble(parts[0].trim());
                double angleDeg = Double.parseDouble(parts[1].trim());
                double rpm = Double.parseDouble(parts[2].trim());
                table.put(distance, new ShotData(angleDeg, rpm));
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to load shooter lookup table: " + file.getAbsolutePath(), e.getStackTrace());
        }

        return table;
    }

    public static ShotData interpolate(double distanceMeters, NavigableMap<Double, ShotData> table) {
        if (table.isEmpty()) {
            return new ShotData(0.0, 0.0);
        }

        Map.Entry<Double, ShotData> lower = table.floorEntry(distanceMeters);
        Map.Entry<Double, ShotData> upper = table.ceilingEntry(distanceMeters);

        if (lower == null) {
            return upper.getValue();
        }
        if (upper == null) {
            return lower.getValue();
        }
        if (upper.getKey().equals(lower.getKey())) {
            return lower.getValue();
        }

        double t = (distanceMeters - lower.getKey()) / (upper.getKey() - lower.getKey());
        double angle = lower.getValue().angleDeg
            + t * (upper.getValue().angleDeg - lower.getValue().angleDeg);
        double rpm = lower.getValue().flywheelRpm
            + t * (upper.getValue().flywheelRpm - lower.getValue().flywheelRpm);

        return new ShotData(angle, rpm);
    }
}
