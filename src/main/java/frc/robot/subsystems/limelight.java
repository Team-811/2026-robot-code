 package frc.robot.subsystems;

 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.networktables.NetworkTableInstance;
 import edu.wpi.first.networktables.BooleanPublisher;
 import edu.wpi.first.networktables.DoublePublisher;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.networktables.DoubleArrayPublisher;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.networktables.NetworkTableInstance;
 import edu.wpi.first.networktables.DoubleArrayPublisher;
 import frc.robot.LimelightHelpers;



 /**
  * Minimal Limelight helper for AprilTag targeting.
  * Reads NT entries each loop (tx/ty/ta + camera-space pose), computes simple distance/height
  * estimates, and publishes telemetry for debugging.
  */
 public class limelight extends SubsystemBase {
    private static final String TABLE_NAME = "limelight-l";  //change if your LL name differs
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);

     //Cached values (meters/degrees) for telemetry and commands.
    private double tx;
    private double ty;
    private double ta;
    private double targetPoseZ;
    private double targetPoseX;
    private double targetPoseY;
    private double targetPoseYaw;
    private boolean hasTarget;

    
  @Override
  public void periodic() {
      System.out.println(
    NetworkTableInstance.getDefault()
      .getTable("limelight-l")
      .getEntry("tx")
      .getDouble(999)
  );

    hasTarget = table.getEntry("tv").getDouble(0) == 1;

    tx = table.getEntry("tx").getDouble(6.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

    if (hasTarget) {
      double[] pose = table
          .getEntry("targetpose_cameraspace")
          .getDoubleArray(new double[6]);

      if (pose.length == 6) {
        targetPoseX = pose[0];
        targetPoseY = pose[1];
        targetPoseZ = pose[2];
        targetPoseYaw = pose[5];

       if (hasTarget) {
    Pose3d targetPose =
        LimelightHelpers.getTargetPose3d_CameraSpace("limelight-l");
    publishTargetPose3d(targetPose);
  }

      }
    }

    publishTelemetry();
  }

    private final DoubleArrayPublisher robotPose2dPub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("/Elastic/RobotPose2d")
          .publish();
  private void publishPose2d(Pose2d pose) {
      robotPose2dPub.set(new double[] {
          pose.getX(),
          pose.getY(),
          pose.getRotation().getRadians()
      });
  }

  
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

  private final DoublePublisher targetPoseXPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/TargetPoseX_m")
          .publish();

  private final DoublePublisher targetPoseYPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/TargetPoseY_m")
          .publish();

  private final DoublePublisher targetPoseZPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/TargetPoseZ_m")
          .publish();

  private final DoublePublisher targetPoseYawPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/TargetPoseYaw_rad")
          .publish();

  private final DoublePublisher rangePub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/Range_m")
          .publish();

  private final DoublePublisher heightOffsetPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Elastic/Limelight/HeightOffset_m")
          .publish();

  private void publishTelemetry() {
    hasTargetPub.set(hasTarget);
    txPub.set(tx);
    tyPub.set(ty);
    taPub.set(ta);

    targetPoseXPub.set(targetPoseX);
    targetPoseYPub.set(targetPoseY);
    targetPoseZPub.set(targetPoseZ);
    targetPoseYawPub.set(targetPoseYaw);

    rangePub.set(getRangeMeters());
    heightOffsetPub.set(getHeightOffsetMeters());
  }
    
  private final DoubleArrayPublisher robotPose3dPub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("/Elastic/RobotPose3d")
          .publish();

  private final DoubleArrayPublisher targetPose3dPub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("/Elastic/TargetPose3d")
          .publish();
  private void publishRobotPose3d(Pose3d pose) {
      robotPose3dPub.set(new double[] {
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          pose.getRotation().getX(),  // roll
          pose.getRotation().getY(),  // pitch
          pose.getRotation().getZ()   // yaw
      });
  }

  private void publishTargetPose3d(Pose3d pose) {
      targetPose3dPub.set(new double[] {
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          pose.getRotation().getX(),
          pose.getRotation().getY(),
          pose.getRotation().getZ()
      });
  }


    public boolean hasTarget() { return hasTarget; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTargetPoseZ() { return targetPoseZ; }
    public double getTargetPoseX() { return targetPoseX; }
    public double getTargetPoseY() { return targetPoseY; }
    public double getTargetPoseYaw() { return targetPoseYaw; }

    /** Approximate straight-line distance from camera to target (meters) using camera-space pose. */
    public double getRangeMeters() { return Math.hypot(targetPoseX, targetPoseZ); }

    /** Approximate vertical offset (meters) from camera to target center. */
    public double getHeightOffsetMeters() { return targetPoseY; }
  
 }
