package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Controlador PID
    private final ProfiledPIDController pivotPID;

    public Shooter(ShooterIO io) {
        this.io = io;
        
        // 1. Configuración del PID
        // Velocidad Max: 100 grados/s, Aceleración Max: 80 grados/s^2
        pivotPID = new ProfiledPIDController(
            0.04, 0.0, 0.0, 
            new TrapezoidProfile.Constraints(100, 80)
        );
        
        pivotPID.setTolerance(2.0);

        // 2. ¡TU SUGERENCIA! Enviamos el controlador completo al Dashboard.
        // Esto creará un widget donde podrás editar P, I, D y ver el error.
        SmartDashboard.putData("Shooter/Pivot_PID", pivotPID);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        
        // Ya no necesitas leer P, I, D aquí. SmartDashboard lo hace solo.
        
        // Opcional: Publicar la medición actual para que el widget sepa dónde está el brazo
        // (A veces el widget necesita ayuda para saber la posición actual "Measurement")
        SmartDashboard.putNumber("Shooter/Pivot_CurrentAngle", inputs.pivotPositionDegrees);
    }

    public Command setPivotAngleCommand(double targetDegrees) {
        return this.run(() -> {
            // Calculamos la salida. 
            // pivotPID usa internamente las ganancias (P,I,D) que tenga configuradas en ese momento.
            double pidOutput = pivotPID.calculate(inputs.pivotPositionDegrees, targetDegrees);
            
            // FeedForward simple para gravedad (opcional)
            double feedforward = 0.0; 
            
            io.setPivotVoltage(pidOutput + feedforward);
        })
        .finallyDo(() -> io.stopPivot());
    }

    // Comando para resetear encoder
    public Command resetPivotEncoder() {
        return runOnce(() -> io.setPivotZero());
    }

    // Comandos Flywheel...
    public Command runShooter(double rpm) {
        return run(() -> io.setFlywheelVelocity(rpm));
    }
    
    public Command stopShooter() {
        return runOnce(() -> io.stopFlywheel());
    }
}