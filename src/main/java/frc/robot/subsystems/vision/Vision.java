package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveBase;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final SwerveBase swerve;

    public Vision(VisionIO io, SwerveBase swerve) {
        this.io = io;
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        // 1. Leer datos de la capa IO
        io.updateInputs(inputs);
        
        // 2. Registrar en AdvantageScope
        Logger.processInputs("Vision", inputs);

        // 3. Si tenemos un target válido y una pose estimada, actualizamos el Swerve
        if (inputs.hasTarget) {
            // Opcional: Aquí podrías filtrar si la pose es muy loca o si la velocidad es muy alta
            
            // Enviamos la corrección al estimador de pose del Swerve
            swerve.addVisionMeasurement(inputs.estimatedPose, inputs.timestamp);
        }
    }
}
