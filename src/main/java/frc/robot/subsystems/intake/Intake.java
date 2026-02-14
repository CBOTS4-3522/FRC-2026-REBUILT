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
    private final ProfiledPIDController pidController;
    private double objetivoGrados; // La meta actual del brazo

    // VARIABLES TUNEABLES
    private double kP = Constants.Intake.kP;
    private double kG = Constants.Intake.kG;
    private double kI = Constants.Intake.kI;
    private double kD = Constants.Intake.kD;

    // VARIABLES PARA CONTAR PELOTAS
    private int contadorPelotas = 0;
    private final Trigger pelotaEntrando;

    // OFFSET
    private double encoderOffset = Constants.Intake.kEncoderOffset;

    public Intake(IntakeIO io) {
        this.io = io;

        // INICIALIZACIÓN PID
        pidController = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(500, 500) // Max Vel, Max Acel
        );

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

        // Dashboard
        SmartDashboard.putNumber("Intake/kP", kP);
        SmartDashboard.putNumber("Intake/kG", kG);
        SmartDashboard.putNumber("Intake/kI", kI);
        SmartDashboard.putNumber("Intake/kD", kD);
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

        // 2. Guardarlos en el log
        Logger.processInputs("Intake", inputs);

        if (!inicializado) {
            // Capturamos la posición real ACTUAL como objetivo
            objetivoGrados = leerEncoder();
            pidController.reset(objetivoGrados);

            inicializado = true; // ¡Listo! Ya no entramos aquí de nuevo
        }

        // 1. Tunear PID en vivo
        double pDashboard = SmartDashboard.getNumber("Intake/kP", kP);
        double gDashboard = SmartDashboard.getNumber("Intake/kG", kG);
        double iDashboard = SmartDashboard.getNumber("Intake/kI", kI);
        double dDashboard = SmartDashboard.getNumber("Intake/kD", kD);

        if (pDashboard != kP || gDashboard != kG || iDashboard != kI || dDashboard != kD) {
            kP = pDashboard;
            kG = gDashboard;
            kI = iDashboard;
            kD = dDashboard;
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
        }

        double posicionActual = leerEncoder();

        boolean bajando = objetivoGrados == Constants.Intake.kTargetDown;

        boolean estaAbajo = posicionActual < (Constants.Intake.kTargetDown + Constants.Intake.kTolerancyDegrees);

        double voltajePID = pidController.calculate(posicionActual, objetivoGrados);

        double feedforward = kG * Math.cos(Math.toRadians(pidController.getSetpoint().position));

        if (bajando && estaAbajo) {
            io.stopBrazo();

        } else {
            io.setVoltajeBrazo(voltajePID + feedforward);
        }
        // Telemetría
        Logger.recordOutput("Intake/Brazo/AnguloActual", leerEncoder());
        Logger.recordOutput("Intake/Rodillos/Contador", contadorPelotas);
        Logger.recordOutput("Intake/Brazo/AnguloReal", inputs.brazoEncoderRaw * 360);
        Logger.recordOutput("Intake/Brazo/AnguloInvertido", (1 - inputs.brazoEncoderRaw) * 360);
        Logger.recordOutput("Intake/Brazo/Objetivo", objetivoGrados);
        Logger.recordOutput("Intake/Brazo/Descanso", bajando && estaAbajo);
    }
}