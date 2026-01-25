package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        
        // Cargar el mapa del campo (AprilTags)
        AprilTagFieldLayout fieldLayout = null;
        try {
            // Actualizado para la temporada 2025 Reefscape
            fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        } catch (Exception e) {
            // Manejar error si no carga el mapa
            e.printStackTrace();
        }

        // Configurar el estimador de PhotonVision
        if (fieldLayout != null) {
            poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                camera, 
                robotToCamera
            );
        } else {
            poseEstimator = null;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Leer resultado crudo de la cámara
        PhotonPipelineResult result = camera.getLatestResult();
        inputs.hasTarget = result.hasTargets();
        inputs.latency = result.getLatencyMillis();

        if (poseEstimator != null) {
            // Calcular la posición del robot basada en los tags visibles
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            
            if (estimatedPose.isPresent()) {
                EstimatedRobotPose pose = estimatedPose.get();
                inputs.estimatedPose = pose.estimatedPose.toPose2d();
                inputs.timestamp = pose.timestampSeconds;
                
                // Guardar IDs de los tags vistos para debugging
                int[] ids = new int[pose.targetsUsed.size()];
                for (int i = 0; i < ids.length; i++) {
                    ids[i] = pose.targetsUsed.get(i).getFiducialId();
                }
                inputs.tagIds = ids;
            } else {
                inputs.hasTarget = false; // Si el estimador falla, no tenemos target válido para odometría
            }
        }
    }
}
