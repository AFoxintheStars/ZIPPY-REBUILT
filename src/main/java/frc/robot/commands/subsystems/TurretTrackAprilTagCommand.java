package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.subsystems.turret.HoodSubsystem;
import frc.robot.subsystems.turret.TurretRotationSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TurretTrackAprilTagCommand extends Command
{
  private final TurretRotationSubsystem turret;
  private final HoodSubsystem hood;

  public TurretTrackAprilTagCommand(TurretRotationSubsystem turret, HoodSubsystem hood)
  {
    this.turret = turret;
    this.hood = hood;
    addRequirements(turret, hood);
  }

  @Override
  public void execute()
  {
    PhotonTrackedTarget target = getBestAllianceTarget();

    if (target == null)
    {
      turret.stop();
      hood.clearTargetAngle();
      hood.stop();
      SmartDashboard.putBoolean("Turret/TrackingTagFound", false);
      SmartDashboard.putNumber("Turret/TrackingDistanceMeters", -1.0);
      return;
    }

    SmartDashboard.putBoolean("Turret/TrackingTagFound", true);
    SmartDashboard.putNumber("Turret/TrackedTagId", target.getFiducialId());
    SmartDashboard.putNumber("Turret/TrackedTagYawDeg", target.getYaw());
    double distanceMeters = target.getBestCameraToTarget().getTranslation().getNorm();
    SmartDashboard.putNumber("Turret/TrackingDistanceMeters", distanceMeters);
    hood.setTargetAngleFromDistance(distanceMeters);
    SmartDashboard.putNumber("Hood/LookupDistanceMeters", distanceMeters);
    SmartDashboard.putNumber("Hood/LookupTargetAngle", hood.getLookupAngle(distanceMeters));

    double yawErrorDeg = target.getYaw();
    if (Math.abs(yawErrorDeg) <= Constants.VisionConstants.TURRET_AIM_TOLERANCE_DEG)
    {
      turret.stop();
      return;
    }

    double speedCmd = yawErrorDeg * Constants.VisionConstants.TURRET_TRACK_KP;
    speedCmd += Math.copySign(Constants.VisionConstants.TURRET_TRACK_KS, speedCmd);
    speedCmd = Math.max(-Constants.VisionConstants.TURRET_TRACK_MAX_SPEED,
                        Math.min(Constants.VisionConstants.TURRET_TRACK_MAX_SPEED, speedCmd));

    boolean tryingPastRight = speedCmd > 0 && turret.atRightLimit();
    boolean tryingPastLeft  = speedCmd < 0 && turret.atLeftLimit();

    if (tryingPastRight || tryingPastLeft)
    {
      turret.stop();
      SmartDashboard.putBoolean("Turret/TrackingAtSoftLimit", true);
      return;
    }

    SmartDashboard.putBoolean("Turret/TrackingAtSoftLimit", false);
    turret.setSpeed(speedCmd);
  }

  private PhotonTrackedTarget getBestAllianceTarget()
  {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    int[] allowedTags = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
        ? Constants.VisionConstants.RED_HUB_TAGS
        : Constants.VisionConstants.BLUE_HUB_TAGS;

    PhotonPipelineResult latest = getLatestCameraResult();
    if (!latest.hasTargets())
    {
      return null;
    }

    PhotonTrackedTarget bestTarget = null;
    double bestAbsYaw = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : latest.getTargets())
    {
      if (!isAllowedTag(target.getFiducialId(), allowedTags))
      {
        continue;
      }

      double absYaw = Math.abs(target.getYaw());
      if (absYaw < bestAbsYaw)
      {
        bestAbsYaw = absYaw;
        bestTarget = target;
      }
    }

    return bestTarget;
  }

  private PhotonPipelineResult getLatestCameraResult()
  {
    var unread = Cameras.TURRET_CAM.camera.getAllUnreadResults();
    if (!unread.isEmpty())
    {
      return unread.get(unread.size() - 1);
    }

    return Cameras.TURRET_CAM.camera.getLatestResult();
  }

  private boolean isAllowedTag(int targetId, int[] allowedTags)
  {
    for (int tagId : allowedTags)
    {
      if (tagId == targetId)
      {
        return true;
      }
    }
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    turret.stop();
    hood.clearTargetAngle();
    hood.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
