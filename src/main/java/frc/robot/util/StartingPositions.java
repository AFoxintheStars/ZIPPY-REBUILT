package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class StartingPositions {

    public static final Pose2d CENTERHUB =
        new Pose2d(new Translation2d(3.5, 4.0), Rotation2d.fromDegrees(0));

    public static final Pose2d LEFTBUMP =
        new Pose2d(new Translation2d(3.5, 5.5), Rotation2d.fromDegrees(0));

    public static final Pose2d RIGHTBUMP =
        new Pose2d(new Translation2d(3.5, 2.5), Rotation2d.fromDegrees(0));

    public static final Pose2d LEFTTRENCH =
        new Pose2d(new Translation2d(3.5, 7.5), Rotation2d.fromDegrees(0));

    public static final Pose2d RIGHTTRENCH =
        new Pose2d(new Translation2d(3.5, 0.5), Rotation2d.fromDegrees(0));
}