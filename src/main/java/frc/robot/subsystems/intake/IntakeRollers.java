package frc.robot.subsystems.intake;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class IntakeRollers extends SubsystemBase {

    // DECLARACION DE IO
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
   

  

    // VARIABLES PARA CONTAR PELOTAS
    private int contadorPelotas = 0;
    private final Trigger pelotaEntrando;

    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;

        // Definicion del trigger
        pelotaEntrando = new Trigger(
                () -> inputs.rodillosCorriente > Constants.Intake.kUmbralCorriente &&
                        Math.abs(inputs.rodillosVelocidad) > 500);

        Trigger corrienteEstable = pelotaEntrando.debounce(0.1);// si dura mas de
        // 0.1s cuenta el pico

        corrienteEstable.onTrue(
                runOnce(
                        () -> {
                            contadorPelotas++;
                        }));


        // // CONFIGURACIÓN DE SYSID (Brazo)
        // m_sysIdRoutine = new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //                 Volts.per(Second).of(1.0), // Rampa suave de 1V por segundo
        //                 Volts.of(6.0), // CUIDADO: Límite de 6V para no romper el mecanismo
        //                 Seconds.of(10), // Tiempo máximo
        //                 null),
        //         new SysIdRoutine.Mechanism(
        //                 (Voltage volts) -> io.setVoltajeBrazo(volts.in(Volts)),
        //                 log -> {
        //                     log.motor("intake-brazo")
        //                             .voltage(Volts.of(inputs.brazoVoltajeAplicado))
        //                             .angularPosition(Degrees.of(inputs.brazoPosicionGrados))
        //                             .angularVelocity(DegreesPerSecond.of(inputs.brazoVelocidadGradosPorSeg));
        //                 },
        //                 this));

    }

    // COMMANDOS DE SYSID
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutine.dynamic(direction);
    // }

    

    // --- MÉTODOS AUXILIARES ---


    public Trigger getTriggerPelota() {
        return this.pelotaEntrando;
    }

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

    public Command ON(){
        return this.runOnce(
            ()-> io.setVoltajeRodillos(6)
        );
    }

    public Command OFF(){
        return this.runOnce(
            ()-> io.setVoltajeRodillos(0)
        );
    }

    public Command movimiento(){
        return Commands.sequence(
            ON(),
            Commands.waitSeconds(0.1),
            OFF(),
            Commands.waitSeconds(1)
            
        
            
        ).repeatedly().finallyDo(
            ()->io.stopRodillos()
        );
    }

    // public Command pasandopelotas(){
    //     return Commands.sequence(
    //         movimiento().withTimeout(5),
    //         encender()
    //     );
    // }


    // --- BUCLE DE CONTROL (EL CEREBRO) ---

    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Rollers", inputs);

    }
}