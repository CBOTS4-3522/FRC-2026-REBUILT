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

public class IntakeLift extends SubsystemBase {

    // DECLARACION DE IO
    private final IntakeLiftIO io;
    private final IntakeLiftIOInputsAutoLogged inputs = new IntakeLiftIOInputsAutoLogged();
    private final SysIdRoutine m_sysIdRoutine;

    // DEFINICIÓN DE HARDWARE
    private double encoderOffset = Constants.Intake.kEncoderOffset;

    // CONTROL BRAZO
    private double objetivoGrados; // La meta actual del brazo

    private double kG = Constants.Intake.kG;

    private double kP = Constants.Intake.kP;
    private double kD = Constants.Intake.kD;
    private double kI = Constants.Intake.kI;

    public IntakeLift(IntakeLiftIO io) {
        this.io = io;

       

        SmartDashboard.putNumber("Intake/kG", kG);
        // Publicar PID inicial al dashboard
        SmartDashboard.putNumber("Intake/kP", kP);
        SmartDashboard.putNumber("Intake/kI", kI);
        SmartDashboard.putNumber("Intake/kD", kD);

        // CONFIGURACIÓN DE SYSID (Brazo)
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0), // Rampa suave de 1V por segundo
                        Volts.of(6.0), // CUIDADO: Límite de 6V para no romper el mecanismo
                        Seconds.of(10), // Tiempo máximo
                        null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.setVoltajeBrazo(volts.in(Volts)),
                        log -> {
                            log.motor("intake-brazo")
                                    .voltage(Volts.of(inputs.brazoVoltajeAplicado))
                                    .angularPosition(Degrees.of(inputs.brazoPosicionGrados))
                                    .angularVelocity(DegreesPerSecond.of(inputs.brazoVelocidadGradosPorSeg));
                        },
                        this));

    }

    // COMMANDOS DE SYSID
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    // COMANDOS ELASTIC
    public Command controlPorSlider() {
        return Commands.sequence(
            // 1. Al iniciar, empatamos el slider con la realidad para evitar un latigazo
            this.runOnce(() -> {
                SmartDashboard.putNumber("Intake/Slider_Objetivo", leerEncoder());
            }),
            // 2. Bucle continuo persiguiendo el número del dashboard
            this.run(() -> {
                double setpoint = SmartDashboard.getNumber("Intake/Slider_Objetivo", leerEncoder());
                setObjetivo(setpoint);
            })
        );
    }

    // --- MÉTODOS AUXILIARES ---

    public void setZero() {
        // Vaciado intencionalmente: El offset ahora se maneja en IntakeIOReal
        // (Hardware)
    }

    // Establece objetivos
    public void setObjetivo(double grados) {
        // Limites de seguridad
        this.objetivoGrados = MathUtil.clamp(grados, 0, 100);
    }


    public boolean estaEnMeta() {
        return Math.abs(leerEncoder() - objetivoGrados) < Constants.Intake.kTolerancyDegrees;
    }

    // --- COMANDOS ---


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

    public double leerEncoder() {
        // inputs.brazoPosicionGrados ahora es el valor crudo (ej. 210 a 300)
        // Al restarle tu kEncoderOffset (ej. 210), el bumper vuelve a ser 0.
        return inputs.brazoPosicionGrados - encoderOffset;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double gDashboard = SmartDashboard.getNumber("Intake/kG", kG);
        if (gDashboard != kG)
            kG = gDashboard;

        double pDashboard = SmartDashboard.getNumber("Intake/kP", kP);
        double iDashboard = SmartDashboard.getNumber("Intake/kI", kI);
        double dDashboard = SmartDashboard.getNumber("Intake/kD", kD);

        // Solo enviamos comando por CAN si notamos que cambiaste un número en
        // AdvantageScope/Shuffleboard
        if (pDashboard != kP || iDashboard != kI || dDashboard != kD) {
            kP = pDashboard;
            kI = iDashboard;
            kD = dDashboard;
            io.setPID(kP, kI, kD);
        }

        double posicionActual = leerEncoder(); // Ahora da 0 en el bumper

        // 1. Calculamos la gravedad base (kG)
        double feedforwardVolts = kG * Math.cos(Math.toRadians(posicionActual));

        // 2. Agregamos la fricción estática (kS)
        double error = objetivoGrados - posicionActual;
        double kS = 2.2; // La fricción de tus poleas

        // Usamos una pequeña zona muerta (ej. 1.5 grados) para evitar que 
        // inyecte los 2.2V cuando ya está prácticamente en la meta y empiece a vibrar.
        if (Math.abs(error) > 1.5) { 
            // Math.signum da 1 si sube, o -1 si baja.
            feedforwardVolts += Math.signum(error) * kS; 
        }

        io.setBrazoPosicion(objetivoGrados + encoderOffset, feedforwardVolts);

        if (DriverStation.isDisabled()) {
            // Seteamos la variable directamente (sin clamp) para que no se pelee con los
            // límites
            objetivoGrados = posicionActual;
        }

        // // 2. Comandar al Spark Max
        // if (objetivoGrados == Constants.Intake.kTargetDown && posicionActual < -1.0)
        // {
        // io.stopBrazo(); // Descansar si está recargado o un poco más abajo
        // } else {
        // // Le sumamos el offset de vuelta.
        // // Si quieres ir a 0 (bumper), le manda 210 al Spark Max.
        // io.setBrazoPosicion(objetivoGrados + encoderOffset, feedforwardVolts);
        // }

        // Telemetría
        Logger.recordOutput("Intake/Brazo/AnguloCrudo", inputs.brazoPosicionGrados);
        Logger.recordOutput("Intake/Brazo/AnguloActual", posicionActual);
        Logger.recordOutput("Intake/Brazo/Objetivo", objetivoGrados);
        Logger.recordOutput("Intake/Brazo/FF_Volts", feedforwardVolts);
    }
}