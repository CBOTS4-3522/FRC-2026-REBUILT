package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camara;
    private final PhotonPoseEstimator poseEstimator;
    private final SwerveBase swerve;

    public Vision(SwerveBase swerve) {
        this.swerve = swerve;
        this.camara = new PhotonCamera(Constants.VisionConstants.kCameraName);
        this.poseEstimator = new PhotonPoseEstimator(
                Constants.VisionConstants.kFieldLayout,
                Constants.VisionConstants.kCameraOffset);
    }

    @Override
    public void periodic() {
        for (var result : camara.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            // --- Lógica de dos estrategias ---
            Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);
            
            if (visionEst.isEmpty()) {
                poseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                visionEst = poseEstimator.update(result);
                poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            }

            visionEst.ifPresent(est -> {
                // Filtro de ambigüedad para evitar poses invertidas
                if (result.getTargets().size() == 1 && result.getBestTarget().getPoseAmbiguity() > 0.2) return;

                // Llamada al método que antes te marcaba error
                Matrix<N3, N1> stdDevs = getEstimationStdDevs(
                    result.getTargets().size(), 
                    result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm()
                );

                swerve.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
            });
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(int numTags, double distance) {
        // Multi-tag: Muy confiable
        if (numTags >= 2) {
            return VecBuilder.fill(0.1, 0.1, 0.1); 
        }
        // Un solo tag cerca: Confiable
        if (distance < 3.0) {
            return VecBuilder.fill(0.4, 0.4, 0.8);
        }
        // Lejos: Poco confiable (subimos los valores para que el robot lo ignore un poco)
        return VecBuilder.fill(2.0, 2.0, 5.0);
    }
}