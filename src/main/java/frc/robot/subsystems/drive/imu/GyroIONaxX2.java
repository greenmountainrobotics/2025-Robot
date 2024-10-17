package frc.robot.subsystems.drive.imu;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.SparkFlexOdometryThread;
import java.util.Queue;

public class GyroIONaxX2 implements GyroIO {

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private final Queue<Double> yawQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONaxX2() {
    ahrs.reset();

    yawTimestampQueue = SparkFlexOdometryThread.getInstance().makeTimestampQueue();

    yawQueue = SparkFlexOdometryThread.getInstance().registerSignal(() -> -ahrs.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = ahrs.isConnected();

    inputs.yawVelocityRadPerSec = Rotation2d.fromDegrees(ahrs.getRate()).getRadians();

    inputs.yawPosition = Rotation2d.fromDegrees(-ahrs.getYaw());
    inputs.odometryYawPositions =
        yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    yawQueue.clear();
    yawTimestampQueue.clear();
  }
}
