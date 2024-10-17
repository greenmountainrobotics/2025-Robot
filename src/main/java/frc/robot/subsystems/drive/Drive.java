// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveConstants.TrackWidthX;
import static frc.robot.constants.DriveConstants.TrackWidthY;
import static frc.robot.constants.TunableConstants.KpTheta;
import static frc.robot.constants.TunableConstants.KpTranslation;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.TunableConstants;
import frc.robot.subsystems.drive.imu.GyroIO;
import frc.robot.subsystems.drive.imu.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.util.Alliance;
import frc.robot.util.FieldPoseUtils;
import java.util.Arrays;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.Trajectory;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double DRIVE_BASE_RADIUS = Math.hypot(TrackWidthX / 2.0, TrackWidthY / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  private final ProfiledPIDController translationController;
  private final ProfiledPIDController thetaController;

  private final Field2d smartDashboardField;

  private DriveState driveState = DriveState.NONE;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    SparkFlexOdometryThread.getInstance().start();

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    translationController =
        new ProfiledPIDController(KpTranslation, 0, 0, new TrapezoidProfile.Constraints(5, 5));
    translationController.setTolerance(DriveConstants.DriveTolerance);
    thetaController =
        new ProfiledPIDController(
            TunableConstants.KpTheta,
            0,
            TunableConstants.KdTheta,
            new TrapezoidProfile.Constraints(5, 5));
    thetaController.setTolerance(DriveConstants.ThetaToleranceRad);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    smartDashboardField = new Field2d();
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    /*    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      if (modules[moduleIndex].getDriveCurrentAmps() > 60
          || modules[moduleIndex].getDriveCurrentAmps() == 0) return;
    }*/

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    smartDashboardField.setRobotPose(getPose());
    SmartDashboard.putData("Field", smartDashboardField);

    Logger.recordOutput(
        "Drive/DistanceFromShooter",
        distanceFromPoint(
            FieldPoseUtils.flipTranslationIfRed(FieldConstants.SpeakerCloseSideCenter)));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    chassisSpeeds = speeds;

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public Twist2d getFieldVelocity() {
    Translation2d translationVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    return new Twist2d(translationVelocity.getX(), translationVelocity.getY(), getYawVelocity());
  }

  public double getYawVelocity() {
    return gyroInputs.connected
        ? gyroInputs.yawVelocityRadPerSec
        : chassisSpeeds.omegaRadiansPerSecond;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Drive/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs standard deviations
   */
  public void addVisionMeasurement(
      Pose2d visionPose,
      double timestamp,
      Function<DriveState, Matrix<N3, N1>> visionMeasurementStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs.apply(driveState));
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    void accept(
        Pose2d visionPose,
        double timestamp,
        Function<DriveState, Matrix<N3, N1>> visionMeasurementStdDevs);
  }

  public enum DriveState {
    NONE,
    ALIGNING_TO_SPEAKER,
    ALIGNING_TO_AMP
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TrackWidthX / 2.0, TrackWidthY / 2.0),
      new Translation2d(TrackWidthX / 2.0, -TrackWidthY / 2.0),
      new Translation2d(-TrackWidthX / 2.0, TrackWidthY / 2.0),
      new Translation2d(-TrackWidthX / 2.0, -TrackWidthY / 2.0)
    };
  }

  public ChassisSpeeds calculatePIDVelocity(Pose2d targetPose) {
    return calculatePIDVelocity(targetPose, getPose(), 0, 0, 0);
  }

  public ChassisSpeeds calculatePIDVelocity(
      Pose2d targetPose, Pose2d currentPose, double xFF, double yFF, double thetaFF) {

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double translationVelocityScalar = translationController.calculate(currentDistance, 0.0);

    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(translationVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX() + xFF,
        driveVelocity.getY() + yFF,
        thetaVelocity + thetaFF,
        currentPose.getRotation());
  }

  public double calculatePIDThetaVelocity(double targetThetaRad, double currentThetaRad) {
    return thetaController.calculate(currentThetaRad, targetThetaRad);
  }

  public boolean poseAtSetpoint(Pose2d setpoint) {
    return Math.abs(getPose().getTranslation().getDistance(setpoint.getTranslation()))
            < DriveConstants.DriveTolerance
        && Math.abs(getPose().getRotation().minus(setpoint.getRotation()).getRadians())
            < DriveConstants.ThetaToleranceRad;
  }

  public double distanceFromPoint(Translation2d point) {
    return getPose().getTranslation().getDistance(point);
  }

  public Command runToPose(Supplier<Pose2d> targetPoseSupplier, boolean stop) {
    return new InstantCommand(
                () -> thetaController.reset(getPose().getRotation().getRadians()), this)
        .andThen(
            new RunCommand(
                () -> {
                  var targetPose = targetPoseSupplier.get();
                  Logger.recordOutput("Auto/TargetPose", targetPose);
                  Logger.recordOutput("Auto/Trajectory", getPose(), targetPose);
                  runVelocity(calculatePIDVelocity(targetPose));
                },
                this))
        .until(() -> poseAtSetpoint(targetPoseSupplier.get()))
        .finallyDo(
            () -> {
              if (stop) this.stop();
            });
  }

  public Command runToPose(
      Supplier<Pose2d> targetPoseSupplier, boolean stop, double translationP, double thetaP) {
    return new DeferredCommand(
        () -> {
          var oldTranslationP = translationController.getP();
          var oldThetaP = thetaController.getP();
          return new InstantCommand(
                  () -> {
                    translationController.setP(translationP);
                    thetaController.setP(thetaP);
                  })
              .andThen(runToPose(targetPoseSupplier, stop))
              .andThen(
                  new InstantCommand(
                      () -> {
                        translationController.setP(oldTranslationP);
                        thetaController.setP(oldThetaP);
                      }));
        },
        Set.of(this));
  }

  public Command runToPose(Supplier<Pose2d> targetPoseSupplier) {
    return runToPose(targetPoseSupplier, true);
  }

  public Command followPath(Trajectory trajectoryFile) {
    ChoreoTrajectory trajectory = Choreo.getTrajectory(trajectoryFile.fileName);

    return new SequentialCommandGroup(
        runToPose(() -> FieldPoseUtils.flipPoseIfRed(trajectory.getInitialPose()), false),
        new InstantCommand(
            () -> {
              Logger.recordOutput(
                  "Auto/TargetPose", FieldPoseUtils.flipPoseIfRed(trajectory.getFinalPose()));
              Logger.recordOutput(
                  "Auto/Trajectory",
                  Arrays.stream(trajectory.getPoses())
                      .map(FieldPoseUtils::flipPoseIfRed)
                      .toArray(Pose2d[]::new));
            }),
        Choreo.choreoSwerveCommand(
            trajectory,
            this::getPose,
            (pose2d, trajectoryState) ->
                calculatePIDVelocity(
                    trajectoryState.getPose(),
                    pose2d,
                    trajectoryState.velocityX,
                    trajectoryState.velocityY,
                    trajectoryState.angularVelocity),
            this::runVelocity,
            Alliance::isRed));
  }

  public Command alignToSpeaker() {
    return new InstantCommand(() -> setState(DriveState.ALIGNING_TO_SPEAKER))
        .andThen(
            new DeferredCommand(
                () -> {
                  var angle =
                      angleModulus(
                          getPose()
                              .getTranslation()
                              .minus(
                                  FieldPoseUtils.flipTranslationIfRed(
                                      FieldConstants.SpeakerCloseSideCenter))
                              .getAngle()
                              .getRadians());

                  var targetTranslation =
                      FieldPoseUtils.flipTranslationIfRed(FieldConstants.SpeakerCloseSideCenter)
                          .plus(
                              new Translation2d(
                                      SmartDashboard.getNumber(
                                          "Shooting Distance M",
                                          FieldConstants.SpeakerShootingDistance),
                                      0)
                                  .rotateBy(
                                      Rotation2d.fromRadians(
                                          Alliance.isRed()
                                              ? angle > Math.PI * 5 / 6
                                                  ? angle
                                                  : angle < Math.PI * -5 / 6
                                                      ? angle
                                                      : angle < 0
                                                          ? Math.PI * -5 / 6
                                                          : Math.PI * 5 / 6
                                              : Math.min(
                                                  Math.max(angle, -Math.PI / 6), Math.PI / 6))));

                  var targetPose =
                      new Pose2d(
                          targetTranslation.getX(),
                          targetTranslation.getY(),
                          FieldPoseUtils.flipTranslationIfRed(FieldConstants.SpeakerCloseSideCenter)
                              .minus(targetTranslation)
                              .getAngle()
                              .minus(Rotation2d.fromRadians(Math.PI)));

                  return runToPose(() -> targetPose);
                },
                Set.of(this)))
        .finallyDo(() -> setState(DriveState.NONE));
  }

  public Command alignToNote(Translation2d noteTranslation) {
    return new DeferredCommand(
        () -> {
          var targetTranslation =
              Alliance.isRed()
                  ? noteTranslation.plus(
                      new Translation2d(
                              DriveConstants.WidthWithBumpersX / 2
                                  + FieldConstants.NoteDiameter / 2,
                              0)
                          .rotateBy(getPose().getTranslation().minus(noteTranslation).getAngle()))
                  : noteTranslation.minus(
                      new Translation2d(
                              DriveConstants.WidthWithBumpersX / 2
                                  + FieldConstants.NoteDiameter / 2,
                              0)
                          .rotateBy(getPose().getTranslation().minus(noteTranslation).getAngle()));

          var targetPose =
              new Pose2d(
                  targetTranslation.getX(),
                  targetTranslation.getY(),
                  noteTranslation.minus(targetTranslation).getAngle());

          var startingPose = getPose();

          return runToPose(
                  () ->
                      new Pose2d(
                          startingPose.getX(), startingPose.getY(), targetPose.getRotation()),
                  false)
              .until(
                  () ->
                      Math.abs(targetPose.getRotation().minus(getPose().getRotation()).getRadians())
                          < Math.PI / 6)
              .andThen(runToPose(() -> targetPose));
        },
        Set.of(this));
  }

  public Command alignToAmp() {
    return new InstantCommand(() -> setState(DriveState.ALIGNING_TO_AMP))
        .andThen(
            runToPose(
                () ->
                    FieldPoseUtils.flipPoseIfRed(
                        new Pose2d(
                            FieldConstants.AmpCenter.minus(
                                new Translation2d(DriveConstants.WidthWithBumpersX, 0)
                                    .times(0.5)
                                    .rotateBy(Rotation2d.fromDegrees(90))),
                            FieldConstants.AmpRotation)),
                true,
                KpTranslation * 4,
                KpTheta))
        .finallyDo(() -> setState(DriveState.NONE));
  }

  public Command alignToFrontOfAmp() {
    return new InstantCommand(() -> setState(DriveState.ALIGNING_TO_AMP))
        .andThen(
            runToPose(
                () ->
                    FieldPoseUtils.flipPoseIfRed(
                        new Pose2d(
                            FieldConstants.AmpCenter.minus(
                                new Translation2d(DriveConstants.WidthWithBumpersX, 0)
                                    .times(0.5)
                                    .plus(
                                        new Translation2d(
                                            DriveConstants.WidthWithBumpersX * 2 / 3, 0))
                                    .rotateBy(Rotation2d.fromDegrees(90))),
                            FieldConstants.AmpRotation))))
        .finallyDo(() -> setState(DriveState.NONE));
  }

  public void setState(DriveState state) {
    driveState = state;
  }

  private double characterizationStartingGyro = 0.0;
  private double characterizationAccumRotation = 0.0;
  private double characterizationStartAvgPosition = 0.0;

  public Command wheelRadiusCharacterization() {
    return new DeferredCommand(
        () -> {
          characterizationStartAvgPosition =
              Arrays.stream(modules)
                      .map(Module::getPositionRad)
                      .map(Math::abs)
                      .reduce(0.0, Double::sum)
                  / 4;
          characterizationAccumRotation = 0.0;
          characterizationStartingGyro = gyroInputs.yawPosition.getRadians();

          return new RunCommand(
              () -> {
                runVelocity(new ChassisSpeeds(0, 0, 0.5));

                characterizationAccumRotation +=
                    angleModulus(
                        gyroInputs.yawPosition.getRadians() - characterizationStartingGyro);

                var characterizationAvgPosition =
                    Arrays.stream(modules)
                                .map(Module::getPositionRad)
                                .map(Math::abs)
                                .reduce(0.0, Double::sum)
                            / 4
                        - characterizationStartAvgPosition;

                Logger.recordOutput(
                    "Drive/AvgWheelRadius",
                    characterizationAccumRotation * TrackWidthX / 2 / characterizationAvgPosition);
                Logger.recordOutput("Drive/AvgSwervePosition", characterizationAvgPosition);
                Logger.recordOutput(
                    "Drive/CharacterizationAccumRotation", characterizationAccumRotation);
                Logger.recordOutput("Drive/StartSwervePosition", characterizationStartAvgPosition);

                characterizationStartingGyro = gyroInputs.yawPosition.getRadians();
              },
              this);
        },
        Set.of(this));
  }
}
