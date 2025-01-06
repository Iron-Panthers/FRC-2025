package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] observations = new PoseObservation[0];
  }

  // from EstimatedRobotPose
  public static record PoseObservation(
      double timestamp,
      Pose3d estimatedPose,
      double ambiguity,
      int tagCount,
      double averageDistance) {}

  default void updateInputs(VisionIOInputs inputs) {}
}
