package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {

    // DECLARACION DE IO
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

    }

    // --- MÉTODOS AUXILIARES ---

    // --- COMANDOS ---

    // COMANDOS DEL BRAZO
    public Command tragarPelotas() {
        return this.run(
                () -> io.setVoltajeRodillos(12)).finallyDo(() -> io.stopRodillos());
    }

    public Command escupirPelotas() {
        return this.run(
                () -> io.setVoltajeRodillos(-12)).finallyDo(() -> io.stopRodillos());
    }

    public Command ON() {
        return this.runOnce(
                () -> io.setVoltajeRodillos(6));
    }

    public Command OFF() {
        return this.runOnce(
                () -> io.setVoltajeRodillos(0));
    }

    public Command movimiento() {
        return Commands.sequence(
                ON(),
                Commands.waitSeconds(0.1),
                OFF(),
                Commands.waitSeconds(1)

        ).repeatedly().finallyDo(
                () -> io.stopRodillos());
    }

    // public Command pasandopelotas(){
    // return Commands.sequence(
    // movimiento().withTimeout(5),
    // encender()
    // );
    // }

    // --- BUCLE DE CONTROL (EL CEREBRO) ---

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Rollers", inputs);

    }
}