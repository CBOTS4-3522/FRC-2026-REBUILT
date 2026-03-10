package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Voltage;
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

    private double objetivoRPMLlanta = 0.0;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.shooter.flywheels.kS,
            Constants.shooter.flywheels.kV,
            Constants.shooter.flywheels.kA);

    private double kP = Constants.shooter.flywheels.kP;
    private double kD = Constants.shooter.flywheels.kD;
    private double kI = Constants.shooter.flywheels.kI;
    // Variables para contar pelotas

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;

        /*
         * 1m 2750rpm
         * 
         */
        SmartDashboard.putNumber("Shooter/kP", kP);
        SmartDashboard.putNumber("Shooter/kI", kI);
        SmartDashboard.putNumber("Shooter/kD", kD);

        SmartDashboard.setDefaultNumber("Shooter/RPM_Test", Constants.shooter.flywheels.defaultRPM);

        // -----------------------------------------------------------
        // CONFIGURACIÓN DE SYSID (Flywheel)
        // -----------------------------------------------------------
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0),
                        Volts.of(7.0),
                        Seconds.of(10),
                        null),
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

    // Devuelve TRUE si ya estamos a +-30 RPM de la meta
    public boolean estaEnVelocidad() {
        // Asegurarnos de que no dispare si el objetivo es 0
        if (objetivoRPMLlanta == 0.0)
            return false;

        double rpmReales = inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor;
        double error = Math.abs(objetivoRPMLlanta - rpmReales);

        return error < 30.0; // Tolerancia de 30 RPM
    }

    public void setObjetivoRPM(double rpm) {
        objetivoRPMLlanta = rpm;
        double rpmMotor = rpm / Constants.shooter.flywheels.relationMotor;
        double rpsMotor = rpmMotor / 60.0;
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
        // Mandamos luces de estado para que Elastic las lea
        SmartDashboard.putBoolean("Elastic/Shooter Listo", estaEnVelocidad());
        SmartDashboard.putBoolean("Elastic/Anti-Atasco Activo", detectoBajonPelota());

        double pDashboard = SmartDashboard.getNumber("Shooter/kP", kP);
        double iDashboard = SmartDashboard.getNumber("Shooter/kI", kI);
        double dDashboard = SmartDashboard.getNumber("Shooter/kD", kD);

        // Solo enviamos comando por CAN si notamos que cambiaste un número en
        // AdvantageScope/Shuffleboard
        if (pDashboard != kP || iDashboard != kI || dDashboard != kD) {
            kP = pDashboard;
            kI = iDashboard;
            kD = dDashboard;
            io.setPID(kP, kI, kD);
        }

        SmartDashboard.putNumber("Shooter/RPMObjetivo", objetivoRPMLlanta);
        SmartDashboard.putNumber("Shooter/ObjetivoMotores",
                objetivoRPMLlanta / Constants.shooter.flywheels.relationMotor);
        Logger.recordOutput("Shooter/FlywheelRPM_Real",
                inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor);
    }

    // ==========================================================
    // COMANDOS DE AZIMUTH (Torreta)
    // ==========================================================

    // ==========================================================
    // COMANDOS DE FLYWHEEL & SYSID
    // ==========================================================

    public Command runShooterCommand(double rpmLlanta) {
        return this.run(() -> {
            // 1. GUARDAMOS EL OBJETIVO para que lo vea el dashboard
            objetivoRPMLlanta = rpmLlanta;

            // 2. Las matemáticas del motor
            double rpmMotor = rpmLlanta / (Constants.shooter.flywheels.relationMotor);
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);

            io.setFlywheelVelocity(rpmMotor, ffVolts);
        }).finallyDo(() -> {
            // Cuando el comando se cancele o termine, el objetivo vuelve a 0
            objetivoRPMLlanta = 0.0;
            io.stopFlywheel();
        });
    }

    public Command testShooterDesdeDashboard() {
        return this.run(() -> {
            // A) Leer el número que los mecánicos escribieron en Elastic
            double rpmDeseado = SmartDashboard.getNumber("Shooter/RPM_Test", Constants.shooter.flywheels.defaultRPM);

            // B) Guardamos el objetivo para que tu gráfica de AdvantageScope siga
            // funcionando
            objetivoRPMLlanta = rpmDeseado;

            // C) Las mismas matemáticas perfectas que ya hicimos
            double rpmMotor = rpmDeseado / Constants.shooter.flywheels.relationMotor;
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);

            io.setFlywheelVelocity(rpmMotor, ffVolts);

        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0; // Reiniciamos al soltar el botón
            io.stopFlywheel();
        });
    }

    public boolean detectoBajonPelota() {
        if (objetivoRPMLlanta == 0.0)
            return false;

        double rpmReales = inputs.flywheelVelocityRPMLider * Constants.shooter.flywheels.relationMotor;

        // Calculamos qué tan caídas están las RPM respecto al objetivo
        double bajon = objetivoRPMLlanta - rpmReales;

        // Lo mandamos al dashboard para que veas exactamente cuánto caen al disparar
        SmartDashboard.putNumber("Shooter/BajonRPM", bajon);

        // Si las RPM caen más de 150 de golpe, asumimos que una pelota acaba de pasar.
        // (Durante el arranque, este valor también será alto, lo cual es bueno porque
        // mantendrá el timer reseteado hasta que las llantas lleguen a su velocidad).
        return bajon > 100.0;
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> {
            objetivoRPMLlanta = 0.0; // Reiniciamos el objetivo
            io.stopFlywheel();

        });
    }

    public Command activarManual() {
        return this.runEnd(
                () -> io.setFlywheelVoltage(5),
                () -> io.stopFlywheel());
    }

    public Command dispararConMapa(DoubleSupplier distanciaMetrosSupplier) {
        return this.run(() -> {
            // 1. Obtenemos la distancia actual
            double distanciaActual = distanciaMetrosSupplier.getAsDouble();

            // 2. Le preguntamos al mapa cuántas RPM necesitamos para esa distancia
            double rpmDeseado = mapaRPM.get(distanciaActual);

            // 3. Lo guardamos para que AdvantageScope y estaEnVelocidad() lo vean
            objetivoRPMLlanta = rpmDeseado;

            // 4. Matemáticas de motores (Idénticas a tu otro comando)
            double rpmMotor = rpmDeseado / Constants.shooter.flywheels.relationMotor;
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);

            io.setFlywheelVelocity(rpmMotor, ffVolts);

        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0;
            io.stopFlywheel();
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}