package frc.robot.subsystems.vision.camera;

import java.util.Optional;
import java.util.Set;

import frc.robot.subsystems.vision.VisionResult;

public interface CameraIO {
    public String getName();

    /**
     * The Pose Estimation from this camera.
     * <br>
     * @param allianceFlipped whether or not to flip the pose to match a Blue Alliance origin
     * @return an Optional object that may contain a Pose estimate wrapped in a {@link VisionResult}.
     */
    public Optional<VisionResult> getBotPoseAsVisionResult(boolean allianceFlipped);

    /**
     * @return a Set of all the targets found in the last measurement from this camera
     */
    public Set<Integer> targets();    
}
