/*
 * IntakeRollers.java
 *
 * Subsistema encargado de la recolección y expulsión de piezas del juego.
 * Aprovecha las macros de WPILib2 (Commands.sequence) para crear secuencias
 * temporales de agitación y prevenir atascos mecánicos.
 */
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }

    /** Comando base: Ingesta a máxima potencia. El finallyDo() garantiza la limpieza de estados. */
    public Command tragarPelotas() {
        return this.run(() -> io.setVoltajeRodillos(12))
                .finallyDo(() -> io.stopRodillos());
    }

    /** Comando base: Expulsión a máxima potencia invirtiendo la polaridad. */
    public Command escupirPelotas() {
        return this.run(() -> io.setVoltajeRodillos(-12))
                .finallyDo(() -> io.stopRodillos());
    }

    // --- ACCIONES ATÓMICAS ---
    // Pequeños bloques de construcción ("RunOnce") para componer rutinas complejas.
    public Command ON() {
        return this.runOnce(() -> io.setVoltajeRodillos(6));
    }

    public Command OFF() {
        return this.runOnce(() -> io.setVoltajeRodillos(0));
    }

    /**
     * Patrón de Agitación (Anti-Jamming).
     * Crea un bucle asíncrono infinito que enciende y apaga los rodillos para 
     * acomodar la pieza por fuerza inercial (Duty Cycle simulado por software).
     */
    public Command movimiento() {
        return Commands.sequence(
                ON(),
                Commands.waitSeconds(0.1), // Delay no bloqueante
                OFF(),
                Commands.waitSeconds(1)
        ).repeatedly() // Reinicia la secuencia al finalizar
        .finallyDo(() -> io.stopRodillos()); // Detiene de forma segura si el comando es cancelado (Interrupted)
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Rollers", inputs);
    }
}