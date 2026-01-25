package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants;


public class VisionAlignCommand extends SubsystemBase {
    // 1. Definimos los objetos
    private final PhotonCamera camera;
    private final PIDController turnController;
    private final SwerveBase m_swerve;
    private final XboxController controller;
    

    // 2. Constructor: Aquí "encendemos" los objetos
    public VisionAlignCommand(SwerveBase swerve, XboxController xbox) {
        this.m_swerve = swerve;
        this.controller = xbox;
        
        // Inicializamos la cámara con el nombre que le pusiste en el Dashboard
        this.camera = new PhotonCamera("microsoft_life_cam"); 

        // Configuramos el PID (P, I, D)
        this.turnController = new PIDController(0.05, 0.0, 0.001);
        this.turnController.setTolerance(1.0); // Margen de error de 1 grado
    }

    // 3. El método que se repite (donde pusiste tu lógica)
    public void execute() {
        double rotationSpeed = 0;
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            Optional<PhotonTrackedTarget> target7 = result.getTargets().stream()
                    .filter(t -> t.getFiducialId() == 7)
                    .findFirst();

            if (target7.isPresent()) {
                // El Yaw es el ángulo horizontal hacia el AprilTag
                rotationSpeed = turnController.calculate(target7.get().getYaw(), 0);
            } else {
                rotationSpeed = -controller.getRightX();
            }
        } else {
            rotationSpeed = -controller.getRightX();
        }

        // public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)

        /*
         * 
         * s_Swerve.drive(
            new Translation2d(xFiltered, yFiltered)
                    .times(Constants.Swerve.kMaxSpeed)
                    .times(speedCutoffVal ? 1 : 0.5),
            rotationVal * Constants.Swerve.kMaxAngularVelocity * (speedCutoffVal ? 0.5 : 1),
            !robotCentric,
            true
        );
         */
        // Enviamos los datos al Swerve
        m_swerve.drive(
        new Translation2d(controller.getLeftY(), controller.getLeftX()).times(Constants.Swerve.kMaxSpeed).times(true ? 1.0 : 0.5),
          
            rotationSpeed, 
            false,
            true
        );
    }
}