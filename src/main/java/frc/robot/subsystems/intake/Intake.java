package frc.robot.subsystems.intake;

import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // DECLARACION DE IO
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean inicializado = false;

    // DEFINICIÓN DE HARDWARE

    // CONTROL BRAZO
    private double objetivoGrados; // La meta actual del brazo

    private double kG = Constants.Intake.kG;
    

    // VARIABLES PARA CONTAR PELOTAS
    private int contadorPelotas = 0;
    private final Trigger pelotaEntrando;

    // OFFSET
    private double encoderOffset = Constants.Intake.kEncoderOffset;

    public Intake(IntakeIO io) {
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

        SmartDashboard.putNumber("Intake/kG", kG);
  
    }

    // --- MÉTODOS AUXILIARES ---

    public double leerEncoder() {
        double lecturaRaw = inputs.brazoEncoderRaw;
        double lecturaInvertida = 1 - lecturaRaw;
        double gradosConOffset = lecturaInvertida * 360;
        double gradosReales = gradosConOffset - encoderOffset;
        return gradosReales;
    }

    public void setZero() {
        this.encoderOffset = (1 - inputs.brazoEncoderRaw) * 360;
    }

    // Establece objetivos
    public void setObjetivo(double grados) {
        // Limites de seguridad
        this.objetivoGrados = MathUtil.clamp(grados, 0, 100);
    }

    public Trigger getTriggerPelota() {
        return this.pelotaEntrando;

    }

    public boolean estaEnMeta() {
        return Math.abs(leerEncoder() - objetivoGrados) < Constants.Intake.kTolerancyDegrees;
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

    // Comando universal
    public Command irA(double grados) {
        return runOnce(() -> setObjetivo(grados));
    }

    // Atajos rápidos
    public Command subir() {
        return irA(Constants.Intake.kTargetUp);
    }

    public Command bajar() {
        return irA(Constants.Intake.kTargetDown);
    }

    // Secuencia de masticar
    public Command 
    masticar() {
        return Commands.sequence(
                irA(50), // Sube un poco
                Commands.waitUntil(this::estaEnMeta), // Espera
                irA(10), // Baja
                Commands.waitUntil(this::estaEnMeta) // Espera
        ).repeatedly();
    }

    

    // --- BUCLE DE CONTROL (EL CEREBRO) ---

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Sintonización en vivo (Opcional: Mandar nuevos PID al Spark si cambian)
        double gDashboard = SmartDashboard.getNumber("Intake/kG", kG);
        if (gDashboard != kG) kG = gDashboard;

        // BUCLE DE CONTROL
        double posicionActual = leerEncoder();
        
        // 1. Calcular Feedforward (Gravedad) en el RIO
        // Esto es lo único que el Spark no puede hacer solo
        double feedforwardVolts = kG * Math.cos(Math.toRadians(posicionActual));

        // 2. Comandar al Spark Max
        // Le decimos: "Ve a objetivoGrados usando tu perfil, y suma feedforwardVolts a tu salida"
        if (objetivoGrados == Constants.Intake.kTargetDown && posicionActual < 5.0) {
             io.stopBrazo(); // Descansar si está abajo
        } else {
             io.setBrazoPosicion(objetivoGrados + encoderOffset, feedforwardVolts);
        }

        // Telemetría
        Logger.recordOutput("Intake/Brazo/AnguloActual", posicionActual);
        Logger.recordOutput("Intake/Brazo/Objetivo", objetivoGrados);
        Logger.recordOutput("Intake/Brazo/FF_Volts", feedforwardVolts);
    }
}