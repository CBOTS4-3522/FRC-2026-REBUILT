/*
 * ShooterFlywheels.java
 *
 * Lógica de alto nivel para el propulsor. 
 * Combina un control basado en modelo de planta (FeedForward) con PID. 
 * Incluye un algoritmo transitorio para detectar caídas de RPM, lo que permite inferir 
 * con alta certeza cuándo un proyectil ha abandonado exitosamente el cañón.
 */
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterFlywheels extends SubsystemBase {
    private final ShooterFlywheelsIO io;
    private final ShooterFlywheelsIOInputsAutoLogged inputs = new ShooterFlywheelsIOInputsAutoLogged();
    private final InterpolatingDoubleTreeMap mapaRPM = new InterpolatingDoubleTreeMap();
    private final SysIdRoutine m_sysIdRoutine;
    
    private final Alert alertaEncoderShooterLider = new Alert("¡Falla de Sensor en Motor Líder del Shooter!", AlertType.kError);
    private final Alert alertaEncoderShooterSeguidor = new Alert("¡Falla de Sensor en Motor Seguidor del Shooter!", AlertType.kError);

    private double objetivoRPMLlanta = 0.0;
    
    // FeedForward (kS: Estática, kV: Velocidad, kA: Aceleración).
    // Provee >90% de la energía necesaria basándose en matemáticas, dejando al PID muy poco trabajo.
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.shooter.flywheels.kS,
            Constants.shooter.flywheels.kV,
            Constants.shooter.flywheels.kA);
            
    private double kP = Constants.shooter.flywheels.kP;
    private double kD = Constants.shooter.flywheels.kD;
    private double kI = Constants.shooter.flywheels.kI;

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;
        SmartDashboard.putNumber("Shooter/kP", kP);
        SmartDashboard.putNumber("Shooter/kI", kI);
        SmartDashboard.putNumber("Shooter/kD", kD);
        SmartDashboard.setDefaultNumber("Shooter/RPM_Test", Constants.shooter.flywheels.defaultRPM);

        // Configuración para rutinas de caracterización (SysId)
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(7.0), Seconds.of(10), null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.setFlywheelVoltage(volts.in(Volts)),
                        log -> {
                            log.motor("shooter-flywheel")
                                    .voltage(Volts.of(inputs.flywheelAppliedVolts))
                                    .angularVelocity(RPM.of(inputs.flywheelVelocityRPMLider))
                                    .angularPosition(Rotations.of(0));
                        },
                        this));
    }

    /** 
     * Condición de tolerancia. Retorna True si los motores están dentro 
     * de la banda de aceptación (±30 RPM) para autorizar un disparo consistente.
     */
    public boolean estaEnVelocidad() {
        if (objetivoRPMLlanta == 0.0) return false;
        
        double rpmReales = inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor;
        double error = Math.abs(objetivoRPMLlanta - rpmReales);
        return error < 30.0; 
    }

    /** Calcula el requerimiento total (FeedForward + Setpoint) para la capa de IO. */
    public void setObjetivoRPM(double rpm) {
        objetivoRPMLlanta = rpm;
        double rpmMotor = rpm / Constants.shooter.flywheels.relationMotor;
        double rpsMotor = rpmMotor / 60.0; // Transformación a Rotaciones Por Segundo para el cálculo FF
        
        double ffVolts = feedforward.calculate(rpsMotor);
        io.setFlywheelVelocity(rpmMotor, ffVolts);
    }

    public void detener() {
        objetivoRPMLlanta = 0.0;
        io.stopFlywheel();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Flywheels", inputs);

        SmartDashboard.putBoolean("Elastic/Shooter Listo", estaEnVelocidad());
        SmartDashboard.putBoolean("Elastic/Anti-Atasco Activo", detectoBajonPelota());
        alertaEncoderShooterLider.set(inputs.errorEncoderLider);
        alertaEncoderShooterSeguidor.set(inputs.errorEncoderFollower);

        // Tuning en caliente para lazos PID desde Shuffleboard
        double pDashboard = SmartDashboard.getNumber("Shooter/kP", kP);
        double iDashboard = SmartDashboard.getNumber("Shooter/kI", kI);
        double dDashboard = SmartDashboard.getNumber("Shooter/kD", kD);
        if (pDashboard != kP || iDashboard != kI || dDashboard != kD) {
            kP = pDashboard;
            kI = iDashboard;
            kD = dDashboard;
            io.setPID(kP, kI, kD);
        }

        SmartDashboard.putNumber("Shooter/RPMObjetivo", objetivoRPMLlanta);
        SmartDashboard.putNumber("Shooter/ObjetivoMotores", objetivoRPMLlanta / Constants.shooter.flywheels.relationMotor);
        Logger.recordOutput("Shooter/FlywheelRPM_Real", inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor);
    }

    // ==========================================================
    // LÓGICA DE DETECCIÓN Y COMANDOS
    // ==========================================================

    /**
     * Detección de salida de proyectil mediante análisis transitorio de RPM.
     * Al entrar una pelota, la transferencia de energía causa una caída repentina de velocidad.
     * Si la caída excede 100 RPM, se infiere una interacción física exitosa.
     */
    public boolean detectoBajonPelota() {
        if (objetivoRPMLlanta == 0.0) return false;
        
        double rpmReales = inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor;
        double bajon = objetivoRPMLlanta - rpmReales;
        
        SmartDashboard.putNumber("Shooter/BajonRPM", bajon);
        return bajon > 100.0;
    }

    public Command runShooterCommand(double rpmLlanta) {
        return this.run(() -> {
            objetivoRPMLlanta = rpmLlanta;
            double rpmMotor = rpmLlanta / (Constants.shooter.flywheels.relationMotor);
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);
            io.setFlywheelVelocity(rpmMotor, ffVolts);
        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0;
            io.stopFlywheel();
        });
    }

    public Command testShooterDesdeDashboard() {
        return this.run(() -> {
            double rpmDeseado = SmartDashboard.getNumber("Shooter/RPM_Test", Constants.shooter.flywheels.defaultRPM);
            objetivoRPMLlanta = rpmDeseado;
            
            double rpmMotor = rpmDeseado / Constants.shooter.flywheels.relationMotor;
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);
            io.setFlywheelVelocity(rpmMotor, ffVolts);
        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0; 
            io.stopFlywheel();
        });
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> {
            objetivoRPMLlanta = 0.0;
            io.stopFlywheel();
        });
    }

    public Command activarManual() {
        return this.runEnd(() -> io.setFlywheelVoltage(5), () -> io.stopFlywheel());
    }

    public Command dispararConMapa(DoubleSupplier distanciaMetrosSupplier) {
        return this.run(() -> {
            double distanciaActual = distanciaMetrosSupplier.getAsDouble();
            double rpmDeseado = mapaRPM.get(distanciaActual);
            objetivoRPMLlanta = rpmDeseado;
            
            double rpmMotor = rpmDeseado / Constants.shooter.flywheels.relationMotor;
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);
            io.setFlywheelVelocity(rpmMotor, ffVolts);
        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0;
            io.stopFlywheel();
        });
    }

    // Comandos de caracterización SysId
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}