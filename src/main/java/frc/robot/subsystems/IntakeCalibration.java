package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCalibration extends SubsystemBase {

    // El encoder conectado al DIO 0 de la RIO
    private final DutyCycleEncoder encoderAbsoluto;

    public IntakeCalibration() {
        encoderAbsoluto = new DutyCycleEncoder(0);
        
        // Por defecto el DutyCycleEncoder reporta de 0.0 a 1.0 (una vuelta completa).
        // Le decimos que si pierde conexión, reporte 0.
        encoderAbsoluto.setConnectedFrequencyThreshold(1);
    }

    @Override
    public void periodic() {
        // 1. Lectura CRUDA (Raw): Va de 0.0 a 1.0
        double lecturaCruda = encoderAbsoluto.get();

        // 2. Conversión a Grados del Brazo
        // Como tu relación es 24:48, 1 vuelta de encoder = 180 grados de brazo.
        double gradosBrazo = lecturaCruda * 180.0;

        // Publicamos en SmartDashboard para que tú anotes los valores
        SmartDashboard.putNumber("Calibracion/Raw (0-1)", lecturaCruda);
        SmartDashboard.putNumber("Calibracion/Grados Aprox", gradosBrazo);
        SmartDashboard.putBoolean("Calibracion/Conectado", encoderAbsoluto.isConnected());
    }
}