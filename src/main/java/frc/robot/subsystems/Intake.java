package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Intake extends SubsystemBase {

    // DEFINICIÓN DE HARDWARE
    private final SparkMax motorRodillos;
    private final SparkMax motorBrazo;
    private final DutyCycleEncoder encoder;

    // CONTROL BRAZO
    private final ProfiledPIDController pidController;
    private double objetivoGrados; // La meta actual del brazo

    // VARIABLES TUNEABLES
    private double kP = Constants.Intake.kP;
    private double kG = Constants.Intake.kG;

    // VARIABLES PARA CONTAR PELOTAS
    private int contadorPelotas = 0;
    private final Trigger pelotaEntrando;

    // OFFSET
    private double encoderOffset = Constants.Intake.kEncoderOffset;

    public Intake() {
        // INICIALIZACIÓN PID
        pidController = new ProfiledPIDController(
                kP, 0, 0,
                new TrapezoidProfile.Constraints(500, 500) // Max Vel, Max Acel
        );

        encoder = new DutyCycleEncoder(Constants.Intake.kCanalEncoder);
        encoder.setConnectedFrequencyThreshold(1);

        // Inicializamos el objetivo en la posición actual para que no de un latigazo al
        // prender
        objetivoGrados = leerEncoder();
        pidController.reset(objetivoGrados);

        motorRodillos = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);
        motorBrazo = new SparkMax(Constants.Intake.kMotorID2, MotorType.kBrushed);

        // CONFIGURACION SPARKS
        SparkMaxConfig configRodillos = new SparkMaxConfig();
        configRodillos.idleMode(IdleMode.kCoast); // Rodillos libres
        configRodillos.smartCurrentLimit(40);

        SparkMaxConfig configBrazo = new SparkMaxConfig();
        configBrazo.idleMode(IdleMode.kBrake); // Brazo frenado para que aguante
        configBrazo.smartCurrentLimit(40);
        configBrazo.inverted(false); // En caso que este al revez

        // Configurar motores
        motorRodillos.configure(configRodillos, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Definicion del trigger
        pelotaEntrando = new Trigger(
            () -> motorRodillos.getOutputCurrent() > Constants.Intake.kUmbralCorriente &&
            Math.abs(motorRodillos.getEncoder().getVelocity()) > 500
        );

        Trigger corrienteEstable = pelotaEntrando.debounce(0.1);// si dura mas de 0.1s cuenta el pico

        corrienteEstable.onTrue(
                runOnce(
                        () -> {
                            contadorPelotas++;
                        }));

        // Dashboard
        SmartDashboard.putNumber("Intake/kP", kP);
        SmartDashboard.putNumber("Intake/kG", kG);
    }

    // --- MÉTODOS AUXILIARES ---

    public double leerEncoder() {
        double lecturaRaw = encoder.get();
        double lecturaInvertida = 1 - lecturaRaw;
        double gradosConOffset = lecturaInvertida * 360;
        double gradosReales = gradosConOffset - encoderOffset;
        return gradosReales;
    }

    public void setZero() {
        this.encoderOffset = (1-encoder.get()) * 360;
    }

    // Establece objetivos
    public void setObjetivo(double grados) {
        // Limites de seguridad
        this.objetivoGrados = MathUtil.clamp(grados, 0, 100);
    }

    public Trigger getTriggerPelota(){
        return this.pelotaEntrando;
    
    }

    public boolean estaEnMeta(){
        return Math.abs(leerEncoder() - objetivoGrados) < Constants.Intake.kTolerancyDegrees;
    }

    // --- COMANDOS ---

    // COMANDOS DEL BRAZO
    public Command tragarPelotas() {
        return this.run(
                () -> motorRodillos.set(1)).finallyDo(() -> motorRodillos.stopMotor());
    }

    public Command escupirPelotas() {
        return this.run(
                () -> motorRodillos.set(-1)).finallyDo(() -> motorRodillos.stopMotor());
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
    public Command masticar() {
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
        // 1. Tunear PID en vivo
        double pDashboard = SmartDashboard.getNumber("Intake/kP", kP);
        double gDashboard = SmartDashboard.getNumber("Intake/kG", kG);

        if (pDashboard != kP || gDashboard != kG) {
            kP = pDashboard;
            kG = gDashboard;
            pidController.setP(kP);
        }

        double posicionActual = leerEncoder();

        boolean bajando = objetivoGrados == Constants.Intake.kTargetDown;

        boolean estaAbajo = posicionActual < (Constants.Intake.kTargetDown + Constants.Intake.kTolerancyDegrees);

        double voltajePID = pidController.calculate(posicionActual, objetivoGrados);

        double feedforward = kG * Math.cos(Math.toRadians(pidController.getSetpoint().position));

        if (bajando && estaAbajo) {
            motorBrazo.stopMotor();
          
        } else {
            motorBrazo.setVoltage(voltajePID + feedforward);
        }
        // Telemetría
        SmartDashboard.putNumber("Intake/Brazo/AnguloActual", leerEncoder());
        SmartDashboard.putNumber("Intake/Rodillos/Contador", contadorPelotas);
        SmartDashboard.putNumber("Intake/Brazo/AnguloReal", encoder.get() * 360);
        SmartDashboard.putNumber("Intake/Brazo/AnguloInvertido", (1 - encoder.get()) * 360);
        SmartDashboard.putNumber("Intake/Brazo/Objetivo", objetivoGrados);
        SmartDashboard.putNumber("Intake/Rodillos/Corriente", motorRodillos.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Rodillos/Velocidad", motorRodillos.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake/Brazo/Corriente", motorBrazo.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Brazo/Descanso", bajando && estaAbajo);
    }
}