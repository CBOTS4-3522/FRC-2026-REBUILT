package frc.robot.subsystems.intake;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class IntakeLift extends SubsystemBase {

    // DECLARACION DE IO
    private final IntakeLiftIO io;
    private final IntakeLiftIOInputsAutoLogged inputs = new IntakeLiftIOInputsAutoLogged();
    

    public IntakeLift(IntakeLiftIO io) {
        this.io = io;

        

        

    }

   public Command bajarProtegido() {
        return this.run(() -> {
            io.setVoltajeLift(-12); // Voltaje normal de bajada
        })
        // LA PROTECCIÓN: Si choca contra el piso ANTES de los 0.8s, se apaga.
        // O si ya estaba en el piso al presionar el botón, se apaga en 0.1s.
        .until(() -> inputs.brazoCorriente > 30.0) // (Usa un amperaje un poco más alto aquí)
        .withTimeout(0.8) // Freno de seguridad por tiempo
        .finallyDo(() -> io.stopLift());
    }

    public Command subirProtegido() {
        return this.run(() -> {
            io.setVoltajeLift(12); // Voltaje normal de subida
        })
        // LA PROTECCIÓN: Si choca arriba, corta la energía fuerte y pasa a sostenimiento
        .until(() -> inputs.brazoCorriente > 25.0) 
        .withTimeout(1.0) 
        .finallyDo(() -> io.setVoltajeLift(-0.5)); // Voltaje para que no se caiga
    }

    // El amperaje que consideras un "choque". 
    // Tendrán que ver AdvantageScope para ver a cuánto sube al atorarse (ej. 15 Amperes)
    private final double CORRIENTE_DE_CHOQUE = 15.0; 

    public Command buscarPisoHoming() {
        return this.run(() -> {
            // 1. Bajamos con un voltaje MUY SUAVE (ej. 2 Voltios) para no romper nada
            io.setVoltajeLift(2.0); 
        })
        // 2. El comando se interrumpe SOLITO en el instante en que la corriente sube
        .until(() -> inputs.brazoCorriente > CORRIENTE_DE_CHOQUE) 
        // 3. Cuando se interrumpe (porque chocó), hacemos esto:
        .finallyDo(() -> {
            io.stopLift();      // Apagamos el motor
            io.resetEncoder();   // ¡MAGIA! Ahora el robot sabe exactamente dónde está el piso
        });
    }

    
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

       
        

        

    }
}