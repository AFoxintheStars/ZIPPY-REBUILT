package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.turret.*;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoTurretAimCommand extends Command {

    private final TurretRotationSubsystem turret;
    private final TurretFlywheelSubsystem flywheel;
    private final HoodSubsystem hood;
    private final Vision vision;

    public AutoTurretAimCommand(
        TurretRotationSubsystem turret,
        TurretFlywheelSubsystem flywheel,
        HoodSubsystem hood,
        Vision vision
    ) {
        this.turret = turret;
        this.flywheel = flywheel;
        this.hood = hood;
        this.vision = vision;

        addRequirements(turret, hood);
    }

    @Override
    public void execute() {

        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {

            double yaw = target.getYaw();

            // ================= SHOOT WHILE DRIVING =================
            double distance =
                vision.getDistanceFromAprilTag(target.getFiducialId());

            double leadYaw = calculateLeadYaw(distance, 0.0, 0.0);

            double correctedYaw = yaw + leadYaw;

            // ================= TURRET =================
            double turnSpeed = correctedYaw * 0.01;

            if ((turnSpeed > 0 && turret.atRightLimit()) ||
                (turnSpeed < 0 && turret.atLeftLimit())) {
                turret.stop();
            } else {
                turret.setSpeed(turnSpeed);
            }

            // ================= HOOD =================
            double hoodAngle = calculateHoodAngle(distance);
            controlHoodToAngle(hoodAngle);

        } else {
            returnToZero();
        }

        // ================= FLYWHEEL =================
        flywheel.setRPM(flywheel.getTargetRPM());
    }

    /* ================= TARGETING ================= */

    private PhotonTrackedTarget getBestTarget() {

        int[] validTags;

        if (DriverStation.getAlliance().isPresent() &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

            validTags = Constants.VisionConstants.RED_HUB_TAGS;

        } else {
            validTags = Constants.VisionConstants.BLUE_HUB_TAGS;
        }

        PhotonTrackedTarget bestTarget = null;
        double bestYaw = Double.MAX_VALUE;

        for (int id : validTags) {
            PhotonTrackedTarget t =
                vision.getTargetFromId(id, Cameras.TURRET_CAM);

            if (t != null) {
                double yaw = Math.abs(t.getYaw());

                if (yaw < bestYaw) {
                    bestYaw = yaw;
                    bestTarget = t;
                }
            }
        }

        return bestTarget;
    }

    /* ================= HOOD ================= */

    private void controlHoodToAngle(double targetAngle) {
        double current = hood.getAngle();

        if (Math.abs(current - targetAngle) < 1.0) {
            hood.stop();
        } else if (current < targetAngle) {
            hood.setSpeed(0.4);
        } else {
            hood.setSpeed(-0.4);
        }
    }

    /* ================= ZEROING ================= */

    private void returnToZero() {
        double angle = turret.getAngleDegrees();

        if (Math.abs(angle) < 2.0) {
            turret.stop();
        } else if (angle > 0) {
            turret.setSpeed(-0.2);
        } else {
            turret.setSpeed(0.2);
        }

        hood.stop();
    }

    /* ================= MATH ================= */

    private double calculateHoodAngle(double distance) {
        return interpolate(distance, HoodConstants.HOOD_LOOKUP);
    }

    private double interpolate(double distance, double[][] table) {

        if (distance <= table[0][0]) return table[0][1];

        if (distance >= table[table.length - 1][0])
            return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            double x1 = table[i][0];
            double y1 = table[i][1];
            double x2 = table[i + 1][0];
            double y2 = table[i + 1][1];

            if (distance >= x1 && distance <= x2) {
                double t = (distance - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }

        return table[0][1];
    }

    /* ================= SHOOT WHILE DRIVING ================= */

    private double calculateLeadYaw(double distance, double vx, double vy) {

        double projectileSpeed = 20.0; // m/s (TUNE)
        double time = distance / projectileSpeed;

        double leadX = vx * time;
        double leadY = vy * time;

        return Math.toDegrees(Math.atan2(leadY, distance - leadX));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        hood.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}