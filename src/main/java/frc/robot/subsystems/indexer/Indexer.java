/*
 * Indexer.java
 *
 * Subsistema de la banda transportadora. Implementa el patrón "Command Factory"
 * utilizando el paradigma funcional de Java (Lambdas y Closures) para generar
 * rutinas de comportamiento en línea, minimizando la creación de clases extra.
 */
package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import java.util.function.BooleanSupplier;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    
    public double velocidadBandas = Constants.Indexer.kVelocidadBandas;
    public double velocidadMecanum = Constants.Indexer.kVelocidadMecanum;

    public Indexer(IndexerIO io) {
        this.io = io;
        SmartDashboard.putNumber("Indexer/VelocidadBandas", velocidadBandas);
        SmartDashboard.putNumber("Indexer/VelocidadMecanum", velocidadMecanum);
    }

    // ==========================================================
    // COMMAND FACTORIES (Uso de Lambdas)
    // ==========================================================

    /**
     * Construye un comando contínuo usando `this.runEnd()`.
     * Arg 1 (Runnable): Se ejecuta iterativamente cada 20ms (Ciclo Execute).
     * Arg 2 (Runnable): Se ejecuta una vez al finalizar o ser interrumpido (Ciclo End).
     */
    public Command encender() {
        return this.runEnd(() -> {
            // Lectura en vivo desde el Dashboard para Live-Tuning
            double vBandas = SmartDashboard.getNumber("Indexer/VelocidadBandas", velocidadBandas);
            double vMecanum = SmartDashboard.getNumber("Indexer/VelocidadMecanum", velocidadMecanum);
            io.setPorcentajeMotores(vBandas, vMecanum);
        }, () -> io.stopMotores()); // Seguridad: Garantiza que el motor se apague
    }

    public Command encenderAuto() {
        return this.runEnd(() -> {
            double vBandas = 1; // Hardcodeado a máxima potencia para autonomía
            double vMecanum = SmartDashboard.getNumber("Indexer/VelocidadMecanum", velocidadMecanum);
            io.setPorcentajeMotores(vBandas, vMecanum);
        }, () -> io.stopMotores());
    }

    public Command alRevez() {
        return this.runEnd(() -> {
            double vBandas = -SmartDashboard.getNumber("Indexer/VelocidadBandas", velocidadBandas);
            double vMecanum = -SmartDashboard.getNumber("Indexer/VelocidadMecanum", velocidadMecanum);
            io.setPorcentajeMotores(vBandas, vMecanum);
        }, () -> io.stopMotores());
    }

    /**
     * Construye un comando atómico usando `this.runOnce()`.
     * Solo ejecuta la función Lambda en el primer ciclo y termina inmediatamente.
     */
    public Command ON() {
        return this.runOnce(() -> io.setPorcentajeMotores(velocidadBandas, velocidadMecanum));
    }

    public Command OFF() {
        return this.runOnce(() -> io.setPorcentajeMotores(-0.5, -0.5));
    }

    /**
     * ALGORITMO ANTI-ATASCOS CON CLOSURES Y DELEGADOS
     * 
     * @param detectorBajon Un delegado (callback) que retorna un booleano, evaluado en tiempo real.
     *                      Se inyecta desde el Shooter (alta cohesión, bajo acoplamiento).
     */
    public Command dispararConAntiAtasco(BooleanSupplier detectorBajon) {
        // Instanciación del temporizador. 
        // Gracias a que las Lambdas soportan "Closures" en Java, la función anónima 
        // de abajo mantiene acceso a este objeto de memoria local en cada ciclo.
        Timer timerAtasco = new Timer();
        
        return Commands.run(() -> {
            boolean pelotaPaso = detectorBajon.getAsBoolean();
            
            // Si el shooter confirma que la pelota salió, reseteamos el reloj
            if (pelotaPaso) {
                timerAtasco.reset();
            } 
            
            // Si el motor lleva más de 2.5 segundos empujando y la pelota no ha salido...
            if (timerAtasco.hasElapsed(2.5)) {
                // Lógica evasiva: Acciona reversa a máxima potencia (-1)
                io.setPorcentajeMotores(-1, -1);
                
                // Si la reversa duró más de 1 segundo (3.5 - 2.5), se resetea el timer
                // para volver a intentar empujar hacia adelante.
                if (timerAtasco.hasElapsed(3.5)) {
                    timerAtasco.reset();
                }
            } else {
                // Flujo nominal: Avanzar normalmente
                io.setPorcentajeMotores(velocidadBandas, velocidadMecanum);
            }
        }, this)
        // Decoradores de ciclo de vida (Method Chaining)
        .beforeStarting(() -> timerAtasco.restart()) // Arranca el timer justo antes del execute
        .finallyDo(() -> io.stopMotores());          // Detiene motores al salir
    }

    /**
     * COMPOSICIÓN DECLARATIVA
     * Encadena múltiples comandos preexistentes (`ON()`, `OFF()`, esperas asíncronas)
     * en una macro secuencial, y la configura para ejecutarse en bucle (`repeatedly()`).
     */
    public Command movimiento() {
        return Commands.sequence(
            ON(),
            Commands.waitSeconds(2),
            OFF(),
            Commands.waitSeconds(0.1)
        ).repeatedly().finallyDo(() -> io.stopMotores());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
}