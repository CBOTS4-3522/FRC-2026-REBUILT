package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import frc.robot.Constants;

public class Vision extends SubsystemBase {
    // 1. Declaramos el objeto de la cámara 📸
    private final PhotonCamera camara;
    // 1. Declaramos la calculadora aquí arriba también
    private final PhotonPoseEstimator poseEstimator;

    public Vision() {
        camara = new PhotonCamera(Constants.VisionConstants.kCameraName);
        // 2. Y aquí solo la inicializamos (sin volver a poner el "PhotonPoseEstimator"
        // antes)
        poseEstimator = new PhotonPoseEstimator(
                Constants.VisionConstants.kFieldLayout,
                Constants.VisionConstants.kCameraOffset);
    }

    @Override
    public void periodic() {
        // Este método se ejecuta automáticamente cada 20 milisegundos.
        // Aquí es donde leeremos los datos de los AprilTags.
        var result = camara.getLatestResult();

        var estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get();

            Logger.recordOutput("Vision/Posicion", pose.estimatedPose);
            Logger.recordOutput("Vision/Tiempo", pose.timestampSeconds);
        }

    }
}