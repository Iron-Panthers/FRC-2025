package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonvision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;

  public VisionIOPhotonvision(AprilTagFieldLayout fieldLayout, int index) {
    camera = new PhotonCamera("photonvision-" + index);
    estimator =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d()); // FIXME transform
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    PoseObservation[] observations = new PoseObservation[results.size()];

    for (int frameIndex = 0; frameIndex < results.size(); ++frameIndex) {
      var frame = results.get(frameIndex);
      if (!frame.hasTargets()) return;

      Optional<EstimatedRobotPose> optEstimation = estimator.update(frame);
      if (optEstimation.isEmpty()) return;
      EstimatedRobotPose estimation = optEstimation.get();

      double totalDistance = 0;
      for (var target : frame.getTargets()) {
        totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
      }

      // FIXME
      var FIDs = frame.getMultiTagResult().get().fiducialIDsUsed;
      int[] tagIDs = new int[FIDs.size()];
      int i = 0;
      for (int id : FIDs) {
        tagIDs[i++] = id;
      }

      var observation =
          new PoseObservation(
              frame.getTimestampSeconds(),
              estimation.estimatedPose,
              frame.getMultiTagResult().get().estimatedPose.ambiguity,
              tagIDs.length,
              totalDistance / tagIDs.length);
      observations[frameIndex] = observation;
    }

    inputs.observations = observations;
  }
}
