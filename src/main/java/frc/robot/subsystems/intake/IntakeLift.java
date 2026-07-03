/*
 * IntakeLift.java
 *
 * Subsistema lógico del elevador del Intake. 
 * Su principal característica arquitectónica es el uso de "Stall Detection" (Detección de atasco 
 * por monitoreo de picos de corriente) para inferir finales de carrera mecánicos por software,
 * eliminando la necesidad de sensores físicos limit switch.
 */
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLift extends SubsystemBase {
    private final IntakeLiftIO io;
    private final IntakeLiftIOInputsAutoLogged inputs = new IntakeLiftIOInputsAutoLogged();
    
    // Límite de amperaje considerado como "Colisión Física" (Stall)
    private final double CORRIENTE_DE_CHOQUE = 15.0; 

    public IntakeLift(IntakeLiftIO io) {
        this.io = io;
    }

    /**
     * Comando de Despliegue (Bajar).
     * Inyecta voltaje negativo hasta que el sensor de corriente detecta un impacto con el suelo.
     */
    public Command bajarProtegido() {
        return this.run(() -> {
            io.setVoltajeLift(-8); // Voltaje Open-Loop para bajar
        })
        // Condición de Interrupción: El motor choca contra el piso y el amperaje se dispara.
        .until(() -> inputs.brazoCorriente > 15.0) 
        // Límite de tiempo absoluto para evitar quemar el motor si el sensor de corriente falla
        .withTimeout(1.5) 
        // Callback de limpieza: Asegurar que el motor quede sin energía
        .finallyDo(() -> io.stopLift());
    }

    /**
     * Comando de Retracción (Subir).
     * Inyecta voltaje positivo y detecta el tope superior físico mediante consumo de corriente.
     */
    public Command subirProtegido() {
        return this.run(() -> {
            io.setVoltajeLift(12); // Voltaje máximo para vencer la gravedad
        })
        // Condición de interrupción con un umbral más alto por el esfuerzo contra la gravedad
        .until(() -> inputs.brazoCorriente > 25.0) 
        .withTimeout(0.7) 
        // Callback de sostenimiento (Holding Voltage): 
        // Inyecta -0.5V (o el valor de FeedForward kG) para compensar la caída por gravedad.
        .finallyDo(() -> io.setVoltajeLift(-0.5)); 
    }

    /**
     * Comando de Calibración Automática (Homing).
     * Busca suavemente el piso para re-sincronizar el encoder relativo del motor.
     */
    public Command buscarPisoHoming() {
        return this.run(() -> {
            io.setVoltajeLift(2.0); // Movimiento lento de exploración
        })
        .until(() -> inputs.brazoCorriente > CORRIENTE_DE_CHOQUE) 
        .finallyDo(() -> {
            io.stopLift();
            io.resetEncoder(); // Cero absoluto establecido algorítmicamente
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}