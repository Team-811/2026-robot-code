package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class limelight extends SubsystemBase {

  private static final String LIMELIGHT_NAME = "limelight-l";

  // Cached values
  private boolean hasTarget = false;
  private double tx = 0.0;
  private double ty = 0.0;
  private double ta = 0.0;

  private Pose3d targetPoseCam = new Pose3d();

  /* ================= Elastic Publishers ================= */

  private final BooleanPublisher hasTargetPub =
      NetworkTableInstance.getDefault()
          .getBooleanTopic("/Elastic/Limelight/HasTarget")
          .publish();

  private final DoublePublisher txPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/tx")
          .publish();

  private final DoublePublisher tyPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/ty")
          .publish();

  private final DoublePublisher taPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/ta")
          .publish();

  private final DoubleArrayPublisher targetPose3dPub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("/Elastic/Limelight/TargetPose3d")
          .publish();

  private final DoublePublisher rangePub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/Range_m")
          .publish();

  private final DoublePublisher heightOffsetPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/HeightOffset_m")
          .publish();

  /* ================= Periodic ================= */

  @Override
  public void periodic() {
    hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);

    tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
    ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
    ta = LimelightHelpers.getTA(LIMELIGHT_NAME);

    if (hasTarget) {
      targetPoseCam =
          LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_NAME);
    } else {
      targetPoseCam = new Pose3d(); // reset so Elastic updates
    }

    publishElastic();
  }

  /* ================= Publishing ================= */

  private void publishElastic() {
    hasTargetPub.set(hasTarget);
    txPub.set(tx);
    tyPub.set(ty);
    taPub.set(ta);

    targetPose3dPub.set(new double[] {
        targetPoseCam.getX(),
        targetPoseCam.getY(),
        targetPoseCam.getZ(),
        targetPoseCam.getRotation().getX(),  // roll
        targetPoseCam.getRotation().getY(),  // pitch
        targetPoseCam.getRotation().getZ()   // yaw
    });

    rangePub.set(getRangeMeters());
    heightOffsetPub.set(getHeightOffsetMeters());
  }

  /* ================= Public Accessors ================= */

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  /** Straight-line distance camera → target (meters) */
  public double getRangeMeters() {
    return Math.hypot(targetPoseCam.getX(), targetPoseCam.getZ());
  }

  /** Vertical offset camera → target (meters) */
  public double getHeightOffsetMeters() {
    return targetPoseCam.getY();
  }
}
