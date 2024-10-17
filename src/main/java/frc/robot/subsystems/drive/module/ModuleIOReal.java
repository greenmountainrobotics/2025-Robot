// Copyright 2021-2023 FRC 6328
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

package frc.robot.subsystems.drive.module;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.constants.DriveConstants.*;
import static frc.robot.constants.IdConstants.CANId.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.SparkFlexOdometryThread;
import java.util.Queue;

/**
 * Module IO implementation for SparkFlex drive motor controller, SparkFlex turn motor controller
 * (NEO or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOReal implements ModuleIO {

  private final CANSparkFlex driveSparkFlex;
  private final CANSparkFlex turnSparkFlex;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOReal(int index) {
    switch (index) {
      case 0:
        // front left
        driveSparkFlex = new CANSparkFlex(FrontLeftDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(FrontLeftTurnId, kBrushless);
        cancoder = new CANcoder(FrontLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(FrontLeftEncoderOffset); // CALIBRATED
        break;
      case 1:
        // front right
        driveSparkFlex = new CANSparkFlex(FrontRightDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(FrontRightTurnId, kBrushless);
        cancoder = new CANcoder(FrontRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(FrontRightEncoderOffset); // CALIBRATED
        break;
      case 2:
        // back left
        driveSparkFlex = new CANSparkFlex(BackLeftDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(BackLeftTurnId, kBrushless);
        cancoder = new CANcoder(BackLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(BackLeftEncoderOffset); // CALIBRATED
        break;
      case 3:
        // back right
        driveSparkFlex = new CANSparkFlex(BackRightDriveId, kBrushless);
        turnSparkFlex = new CANSparkFlex(BackRightTurnId, kBrushless);
        cancoder = new CANcoder(BackRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(BackRightEncoderOffset); // CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

    /*    driveSparkFlex.restoreFactoryDefaults();
    turnSparkFlex.restoreFactoryDefaults();*/

    driveSparkFlex.setCANTimeout(250);
    turnSparkFlex.setCANTimeout(250);

    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkFlex.getEncoder();

    turnSparkFlex.setInverted(isTurnMotorInverted);
    driveSparkFlex.setSmartCurrentLimit(40);
    turnSparkFlex.setSmartCurrentLimit(30);
    driveSparkFlex.enableVoltageCompensation(12.0);
    turnSparkFlex.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkFlex.setCANTimeout(0);
    turnSparkFlex.setCANTimeout(0);

    driveSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkFlex.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    drivePositionQueue =
        SparkFlexOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkFlexOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkFlex.burnFlash();
    turnSparkFlex.burnFlash();

    timestampQueue = SparkFlexOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DriveConstants.DriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / DriveConstants.DriveGearRatio;
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = driveSparkFlex.getOutputCurrent();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / DriveConstants.TurnGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / DriveConstants.TurnGearRatio;
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkFlex.getOutputCurrent();

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / DriveConstants.DriveGearRatio)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.TurnGearRatio))
            .toArray(Rotation2d[]::new);
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.driveMotorTemperature = driveSparkFlex.getMotorTemperature();
    inputs.turnMotorTemperature = turnSparkFlex.getMotorTemperature();

    drivePositionQueue.clear();
    turnPositionQueue.clear();
    timestampQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
