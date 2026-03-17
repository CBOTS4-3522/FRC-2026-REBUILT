package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation; // IMPORTANTE: Importar DriverStation
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
                
        // Creamos el botón en la pantalla (Apagado por defecto para máxima seguridad)
        SmartDashboard.putBoolean("Elastic/Activar Vision", false);
    }

    @Override
    public void periodic() {
        // ==========================================================
        // 1. BLOQUEO ABSOLUTO EN AUTÓNOMO
        // ==========================================================
        // Si estamos en Autónomo, nos salimos de inmediato. Cero riesgos.
        if (DriverStation.isAutonomousEnabled()) {
            return;
        }

        // ==========================================================
        // 2. KILL SWITCH EN TELEOP (Desde Elastic)
        // ==========================================================
        // Si estamos en Teleop, pero el botón de Elastic está apagado, también nos salimos.
        if (!SmartDashboard.getBoolean("Elastic/Activar Vision", false)) {
            return; 
        }

        // --- A partir de aquí, la cámara ya procesa datos ---
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
                
                // Filtro de ambigüedad
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