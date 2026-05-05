package frc.robot.commands.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.turret.HoodSubsystem;
import frc.robot.subsystems.turret.TurretFlywheelSubsystem;
import frc.robot.subsystems.turret.TurretRotationSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TurretTrackAprilTagCommand extends Command
{
  private final TurretRotationSubsystem turret;
  private final HoodSubsystem hood;
  private final TurretFlywheelSubsystem flywheel;
  private final LoggedTunableNumber turretTrackKp =
      new LoggedTunableNumber("TurretTrack/kP", Constants.VisionConstants.TURRET_TRACK_PID_KP);
  private final LoggedTunableNumber turretTrackKi =
      new LoggedTunableNumber("TurretTrack/kI", Constants.VisionConstants.TURRET_TRACK_PID_KI);
  private final LoggedTunableNumber turretTrackKd =
      new LoggedTunableNumber("TurretTrack/kD", Constants.VisionConstants.TURRET_TRACK_PID_KD);
  private final LoggedTunableNumber turretTrackMaxSpeed =
      new LoggedTunableNumber("TurretTrack/MaxSpeed", Constants.VisionConstants.TURRET_TRACK_MAX_SPEED);

  private final PIDController turretAimPid = new PIDController(
      turretTrackKp.get(),
      turretTrackKi.get(),
      turretTrackKd.get());

  public TurretTrackAprilTagCommand(
      TurretRotationSubsystem turret,
      HoodSubsystem hood,
      TurretFlywheelSubsystem flywheel)
  {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    turretAimPid.setTolerance(Constants.VisionConstants.TURRET_AIM_TOLERANCE_DEG);
    turretAimPid.setIntegratorRange(-0.2, 0.2);
    addRequirements(turret, hood, flywheel);
  }

  @Override
  public void execute()
  {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> turretAimPid.setPID(turretTrackKp.get(), turretTrackKi.get(), turretTrackKd.get()),
        turretTrackKp,
        turretTrackKi,
        turretTrackKd);

    PhotonTrackedTarget target = getBestAllianceTarget();

    if (target == null)
    {
      turret.stop();
      turretAimPid.reset();
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
    flywheel.setRPMFromDistance(distanceMeters);
    SmartDashboard.putNumber("Flywheel/LookupDistanceMeters", distanceMeters);
    SmartDashboard.putNumber("Flywheel/LookupTargetRPM", flywheel.getLookupRPM(distanceMeters));

    double yawErrorDeg = target.getYaw();
    if (Math.abs(yawErrorDeg) <= Constants.VisionConstants.TURRET_AIM_TOLERANCE_DEG)
    {
      turret.stop();
      turretAimPid.reset();
      return;
    }

    double speedCmd = turretAimPid.calculate(yawErrorDeg, 0.0);
    speedCmd = Math.max(-turretTrackMaxSpeed.get(),
                        Math.min(turretTrackMaxSpeed.get(), speedCmd));

    boolean tryingPastRight = speedCmd > 0 && turret.atRightLimit();
    boolean tryingPastLeft  = speedCmd < 0 && turret.atLeftLimit();

    if (tryingPastRight || tryingPastLeft)
    {
      turret.stop();
      turretAimPid.reset();
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
    turretAimPid.reset();
    hood.clearTargetAngle();
    hood.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
