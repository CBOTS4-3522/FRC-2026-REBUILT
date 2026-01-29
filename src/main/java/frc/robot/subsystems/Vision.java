package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.littletonrobotics.junction.Logger; // Asegúrate de importar esto
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camara;
    private final PhotonPoseEstimator poseEstimator;
    private final SwerveBase swerve; // <--- 1. Nueva variable para guardar el chasis

    // 2. Actualizamos el constructor para que pida el Swerve
    public Vision(SwerveBase swerve) {
        this.swerve = swerve; // Guardamos la referencia

        camara = new PhotonCamera(Constants.VisionConstants.kCameraName);
        poseEstimator = new PhotonPoseEstimator(
                Constants.VisionConstants.kFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Usamos la estrategia correcta aquí también
                Constants.VisionConstants.kCameraOffset);
    }

    @Override
    public void periodic() {
        var allResults = camara.getAllUnreadResults();

        for (var result : allResults) {
            // Usamos la estrategia específica para coprocesador
            var estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedPose.isPresent()) {
                var pose = estimatedPose.get();

                // Logs (ya los tenías)
                Logger.recordOutput("Vision/Posicion", pose.estimatedPose);
                Logger.recordOutput("Vision/Tiempo", pose.timestampSeconds);

                // 3. ¡EL PASO FINAL! Enviamos el dato al Swerve
                // Convertimos Pose3d a Pose2d porque el chasis se mueve en 2D
                swerve.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),
                        pose.timestampSeconds);
            }
        }
    }
}