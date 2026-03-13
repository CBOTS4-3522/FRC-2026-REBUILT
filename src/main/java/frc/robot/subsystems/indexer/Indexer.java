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

    public Command encender() {
        return this.runEnd(() -> {
            double vBandas = SmartDashboard.getNumber("Indexer/VelocidadBandas", velocidadBandas);
            double vMecanum = SmartDashboard.getNumber("Indexer/VelocidadMecanum", velocidadMecanum);
            io.setPorcentajeMotores(vBandas, vMecanum);
        }, () -> io.stopMotores());
    }

    public Command encenderAuto() {
        return this.runEnd(() -> {
            double vBandas = 1;
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

    public Command ON() {
        return this.runOnce(() -> io.setPorcentajeMotores(velocidadBandas, velocidadMecanum));
    }

    public Command OFF() {
        return this.runOnce(() -> io.setPorcentajeMotores(-0.5, -0.5));
    }

    public Command dispararConAntiAtasco(BooleanSupplier detectorBajon) {
        Timer timerAtasco = new Timer();
        
        return Commands.run(() -> {
            boolean pelotaPaso = detectorBajon.getAsBoolean();
            
            // Si detectamos un bajón, significa que la pelota logró salir (o que apenas estamos acelerando).
            // ¡Todo está bien! Reseteamos el reloj de atascos.
            if (pelotaPaso) {
                timerAtasco.reset();
            } 
            
            // Si llevamos 2.0 segundos enteros empujando sin que las llantas sufran ningún bajón...
            if (timerAtasco.hasElapsed(2.5)) {
                
                io.setPorcentajeMotores(-1, -1); // REVERSA para desatorar
                
                // Después de 0.3 segundos de reversa, reiniciamos el ciclo para volver a empujar
                if (timerAtasco.hasElapsed(3.5)) {
                    timerAtasco.reset();
                }
            } else {
                // Modo normal: empujar hacia el shooter a toda velocidad
                io.setPorcentajeMotores(velocidadBandas, velocidadMecanum);
            }
            
        }, this)
        .beforeStarting(() -> timerAtasco.restart())
        .finallyDo(() -> io.stopMotores());
    }

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