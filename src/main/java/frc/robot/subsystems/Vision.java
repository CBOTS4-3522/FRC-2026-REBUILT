package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        
        // API 2026: Ya no le pasamos estrategias aquí, solo el mapa de la cancha y dónde está la cámara
        this.poseEstimator = new PhotonPoseEstimator(
                Constants.VisionConstants.kFieldLayout,
                Constants.VisionConstants.kCameraOffset);
    }

    @Override
    public void periodic() {
        for (var result : camara.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            // 1. Intentamos la mejor estrategia directa: MultiTag en el coprocesador
            Optional<EstimatedRobotPose> visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

            // 2. Si falló (porque solo hay 1 tag visible), usamos la estrategia más segura para 1 tag
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            // 3. Si logramos calcular una pose, se la pasamos al chasis
            visionEst.ifPresent(est -> {
                SmartDashboard.putNumber("Vision/Ambigüedad", result.getBestTarget().getPoseAmbiguity());
                
                // Filtro de ambigüedad (Subido a 0.5 para pruebas, luego bájalo a 0.2)
                if (result.getTargets().size() == 1 && result.getBestTarget().getPoseAmbiguity() > 0.5) return;

                Matrix<N3, N1> stdDevs = getEstimationStdDevs(
                        result.getTargets().size(),
                        result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());

                swerve.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
            });
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(int numTags, double distance) {
        if (numTags >= 2) return VecBuilder.fill(0.1, 0.1, 0.1);
        if (distance < 3.0) return VecBuilder.fill(0.4, 0.4, 0.8);
        return VecBuilder.fill(2.0, 2.0, 5.0);
    }
}