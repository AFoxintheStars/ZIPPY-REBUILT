// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.SliderSubsystem;
import frc.robot.subsystems.prefeed.PrefeedSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretFlywheelSubsystem;
import frc.robot.subsystems.turret.TurretRotationSubsystem;

import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer
{
  final         CommandXboxController driverXbox = new CommandXboxController(0);

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private final SliderSubsystem slider = new SliderSubsystem();

  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final PrefeedSubsystem prefeed = new PrefeedSubsystem();

  private final TurretRotationSubsystem turret = new TurretRotationSubsystem();

  private final TurretFlywheelSubsystem flywheel = new TurretFlywheelSubsystem();

  private final SendableChooser<Command> autoChooser;

  private Command rumbleCommand(double strength) {
  return Commands.startEnd(
      () -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, strength),
      () -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)
  );
}

public Command rumblePulse(CommandXboxController controller) {
    return Commands.runEnd(
        () -> controller.getHID().setRumble(
            GenericHID.RumbleType.kBothRumble, 0.4),
        () -> controller.getHID().setRumble(
            GenericHID.RumbleType.kBothRumble, 0)
    ).withTimeout(0.2);
}

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("Example", Commands.print("I AM AN EXAMPLE COMMAND!"));

    autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                                                    .andThen(Commands.none()));

    autoChooser.addOption("Reset Gyro w/ Alliance", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    slider.setDefaultCommand(slider.setHeight(Meters.of(0)));

    if (autoChooser.getSelected() == null ) {
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);


    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));



    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    } else
    {
      
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));

      driverXbox.leftBumper().whileTrue(
          intake.intakeCommand()
              .alongWith(rumbleCommand(0.1))
      );

      driverXbox.rightBumper().whileTrue(
          intake.outtakeCommand()
              .alongWith(rumbleCommand(0.1))
      );
      
 
      driverXbox.button(8).whileTrue(
          prefeed.intake()
              .alongWith(rumbleCommand(0.25))
      );

      driverXbox.button(7).whileTrue(
          prefeed.outtake()
              .alongWith(rumbleCommand(0.25))
      );

      
      // driverXbox.x().whileTrue(slider.setHeight(Meters.of(0.15)));
      // driverXbox.b().whileTrue(slider.setHeight(Meters.of(0)));
      driverXbox.b().whileTrue(slider.set(-0.15));
      driverXbox.x().whileTrue(slider.set(0.15));

      driverXbox.y().whileTrue(prefeed.intake());
      
      driverXbox.povLeft().whileTrue(turret.rotateLeft());
      driverXbox.povRight().whileTrue(turret.rotateRight());

      driverXbox.rightTrigger().whileTrue(
          flywheel.shoot()
              .alongWith(
                  prefeed.intake().onlyIf(flywheel::isReadyToShoot),
                  Commands.waitUntil(flywheel::isReadyToShoot)
                      .andThen(rumblePulse(driverXbox))
              )
      );
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
  Command selected = autoChooser.getSelected();

  if (selected == null) return Commands.none();

  if (selected instanceof com.pathplanner.lib.commands.PathPlannerAuto auto) {
    return auto.beforeStarting(() ->
        drivebase.resetOdometry(auto.getStartingPose())
    );
  }

  return selected;
  }
}