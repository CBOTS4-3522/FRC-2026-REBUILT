package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Nuevo para Torreta
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine m_sysIdRoutine;

    // PID para el Azimuth (Torreta)
    private final ProfiledPIDController azimuthPID;

    // FeedForward para Torreta (Fricción)
    // kS: Voltaje mínimo para que empiece a moverse.
    // kV: Voltaje por velocidad (opcional en posición, pero ayuda).
    private final SimpleMotorFeedforward azimuthFeedforward;

    public Shooter(ShooterIO io) {
        this.io = io;

        // -----------------------------------------------------------
        // 1. CONFIGURACIÓN DEL AZIMUTH (Torreta)
        // -----------------------------------------------------------

        // ADVERTENCIA SOBRE TU REDUCCIÓN 1:24
        // 1:24 es MUY rápido para una torreta (aprox 1400 grados/segundo).
        // Si pones un kP alto, va a oscilar violentamente.
        // He limitado la velocidad a 180 grados/s (media vuelta por segundo) por
        // seguridad.

        azimuthPID = new ProfiledPIDController(
                0.01, 0.0, 0.0, // kP empieza MUY bajo por la velocidad del mecanismo
                new TrapezoidProfile.Constraints(180, 100) // MaxVel (deg/s), MaxAcel (deg/s^2)
        );
        azimuthPID.setTolerance(1.0); // Tolerancia de 1 grado

        // Feedforward para torreta:
        // kS (Static Friction): Empieza con 0.1 o 0.15. Es lo que necesita para romper
        // la fricción.
        // kV (Velocity): 0.0 por ahora, úsalo si ves que le cuesta mantener velocidad.
        azimuthFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

        // Publicar PID en Elastic
        SmartDashboard.putData("Shooter/Azimuth_PID", azimuthPID);

        // -----------------------------------------------------------
        // 2. CONFIGURACIÓN DE SYSID (Flywheel) - IGUAL
        // -----------------------------------------------------------
        m_sysIdRoutine = new SysIdRoutine(
                // 1. Configuración de la prueba
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0), // Rampa: Sube 1 voltio por segundo (Quasistatic)
                        Volts.of(7.0), // Escalón: Aplica 7 voltios de golpe (Dynamic)
                        Seconds.of(10), // Timeout: Si dura más de 10s, se apaga (Seguridad)
                        null // Log state: null usa el default (motor voltage)
                ),
                // 2. Mecanismo (Ya lo tenías bien)
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> io.setFlywheelVoltage(volts.in(Volts)),
                        log -> {
                            log.motor("shooter-flywheel")
                                    .voltage(Volts.of(inputs.flywheelAppliedVolts))
                                    .angularVelocity(RPM.of(inputs.flywheelVelocityRPM))
                                    .angularPosition(Rotations.of(0)); // Posición no importa tanto en shooter de
                                                                       // velocidad
                        },
                        this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Importante para que el Widget de PID sepa la posición real
        SmartDashboard.putNumber("Shooter/Azimuth_CurrentAngle", inputs.azimuthPositionDegrees);
    }

    // ==========================================================
    // COMANDOS DE AZIMUTH (Torreta)
    // ==========================================================

    public Command setAzimuthAngleCommand(double targetDegrees) {
        return this.run(() -> {
            // 1. Cálculo del PID (Posición)
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees, targetDegrees);

            // 2. Cálculo del FeedForward (Cinemática)
            // Calculamos qué voltaje necesitamos para ir a la velocidad que el perfil pide
            double setpointVelocity = azimuthPID.getSetpoint().velocity;
            double ffOutput = azimuthFeedforward.calculate(setpointVelocity);

            // 3. Sumar y enviar (PID + FeedForward)
            // NO usamos gravedad (cos/sin) porque es horizontal.
            io.setPivotVoltage(pidOutput + ffOutput);
        })
                .finallyDo(() -> io.stopAzimuth());
    }

    // Reseteo de encoder (útil si la torreta se descuadra)
    public Command resetAzimuthEncoder() {
        return runOnce(() -> io.setPivotZero());
    }

    public Command manualAzimuthCommand(double volts) {
        return run(() -> io.setPivotVoltage(volts));
    }

    // ==========================================================
    // COMANDOS DE FLYWHEEL & SYSID (Igual que antes)
    // ==========================================================

    public Command runShooterCommand(double rpm) {
        return this.run(() -> io.setFlywheelVelocity(rpm));
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> io.stopFlywheel());
    }

    public Command encendidoOpenLoop(){
        return this.runEnd(
            ()->io.setFlywheelVoltage(12), ()->io.setFlywheelVoltage(0)            );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}