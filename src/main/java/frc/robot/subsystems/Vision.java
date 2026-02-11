package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveBase;

// Importaciones necesarias para las matemáticas de confianza
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

public class Vision extends SubsystemBase {
    private final PhotonCamera camara;
    private final PhotonPoseEstimator poseEstimator;
    private final SwerveBase swerve;

    public Vision(SwerveBase swerve) {
        this.swerve = swerve;
        camara = new PhotonCamera(Constants.VisionConstants.kCameraName);
        poseEstimator = new PhotonPoseEstimator(
                Constants.VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.VisionConstants.kCameraOffset);
    }

    @Override
    public void periodic() {
        // Obtenemos todos los resultados pendientes
        var allResults = camara.getAllUnreadResults();

        for (var result : allResults) {
            // Solo procesamos si hay tags visibles
            if (!result.hasTargets()) continue;

            var estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedPose.isPresent()) {
                var pose = estimatedPose.get();

                // --- MEJORA CRÍTICA: FILTRO DE CALIDAD ---
                // No aceptamos poses que estén fuera del campo (ej. ruido fantasma)
                if (pose.estimatedPose.getX() > 17.0 || pose.estimatedPose.getX() < -1.0 ||
                    pose.estimatedPose.getY() > 8.5 || pose.estimatedPose.getY() < -1.0) {
                    continue; 
                }

                // Logs para depuración
                Logger.recordOutput("Vision/Posicion", pose.estimatedPose);
                
                // Calculamos qué tanto confiar en esta medición
                // (Matriz de Desviación Estándar: X, Y, Rotación)
                Matrix<N3, N1> deviation = getEstimationStdDevs(result.targets.size(), 
                                             result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());

                // Enviamos el dato al Swerve CON la confianza calculada
                swerve.swerveOdometer.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds,
                        deviation); // <--- Aquí está la clave de la estabilidad
            }
        }
    }

    /**
     * Calcula la "desconfianza" (Standard Deviation) de la visión.
     * Números grandes = Confiar MENOS (ej. 1 tag lejos).
     * Números chicos = Confiar MÁS (ej. 2 tags cerca).
     */
    private Matrix<N3, N1> getEstimationStdDevs(int numTags, double avgDist) {
        var estStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); // Valor base alto (desconfianza)

        // Si vemos múltiples tags, confiamos mucho más (bajamos la desviación)
        if (numTags >= 2) {
            estStdDevs = VecBuilder.fill(0.05, 0.05, 0.2); 
        } 
        // Si vemos 1 solo tag, dependemos de qué tan lejos esté
        else if (avgDist < 3.0) { // Menos de 3 metros
             // Fórmula empírica: mientras más lejos, más desconfiamos
            estStdDevs = VecBuilder.fill(0.1 * avgDist, 0.1 * avgDist, 1.0); 
        } 
        else {
            // Muy lejos con 1 solo tag -> prácticamente ignorar
            estStdDevs = VecBuilder.fill(1.0, 1.0, 2.0); 
        }

        return estStdDevs;
    }
}