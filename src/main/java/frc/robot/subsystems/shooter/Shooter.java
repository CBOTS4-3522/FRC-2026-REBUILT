package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine m_sysIdRoutine;
    private final ProfiledPIDController azimuthPID;
    private double objetivoRPMLlanta = 0.0;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.shooter.flywheels.kS,
        Constants.shooter.flywheels.kV,
        Constants.shooter.flywheels.kA
    );

    private double kP = Constants.shooter.flywheels.kP;
    private double kD = Constants.shooter.flywheels.kD;
    private double kI = Constants.shooter.flywheels.kI;

    public Shooter(ShooterIO io) {
        this.io = io;

        SmartDashboard.putNumber("Shooter/kP", kP);
        SmartDashboard.putNumber("Shooter/kI", kI);
        SmartDashboard.putNumber("Shooter/kD", kD);

        SmartDashboard.setDefaultNumber("Shooter/RPM_Test", 2000.0);
        // -----------------------------------------------------------
        // CONFIGURACIÓN DEL AZIMUTH (Torreta)
        // -----------------------------------------------------------
        azimuthPID = new ProfiledPIDController(
                0.015, 0.0, 0.0, 
                new TrapezoidProfile.Constraints(180, 100) 
        );
        azimuthPID.setTolerance(1.0); 

        SmartDashboard.putData("Shooter/Azimuth_PID", azimuthPID);

        // -----------------------------------------------------------
        // CONFIGURACIÓN DE SYSID (Flywheel)
        // -----------------------------------------------------------
        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0), 
                        Volts.of(7.0), 
                        Seconds.of(10), 
                        null 
                ),
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
        if (objetivoRPMLlanta == 0.0) return false;

        double rpmReales = inputs.flywheelVelocityRPMLider * 1.5;
        double error = Math.abs(objetivoRPMLlanta - rpmReales);
        
        return error < 30.0; // Tolerancia de 30 RPM
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

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

        SmartDashboard.putNumber("Shooter/Azimuth_CurrentAngle", inputs.azimuthPositionDegrees);
        SmartDashboard.putNumber("Shooter/RPMObjetivo", objetivoRPMLlanta);
        SmartDashboard.putNumber("SHooter/ObjetivoMotores", objetivoRPMLlanta/1.5);
        Logger.recordOutput("Shooter/FlywheelRPM_Real", inputs.flywheelVelocityRPMLider * 1.5);
    }

    // ==========================================================
    // COMANDOS DE AZIMUTH (Torreta)
    // ==========================================================

    public Command setAzimuthAngleCommand(double targetDegrees) {
        return this.run(() -> {
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees, targetDegrees);
            io.setAzimuthVoltage(pidOutput);
        })
        .finallyDo(() -> io.stopAzimuth());
    }

    public Command resetAzimuthEncoder() {
        return runOnce(() -> io.setAzimuthZero());
    }

    public Command manualAzimuthCommand(double volts) {
        return run(() -> io.setAzimuthVoltage(volts));
    }

    // ==========================================================
    // COMANDOS DE PIVOT (Chamfle con Servo)
    // ==========================================================

    public Command setPivotAngleCommand(double targetDegrees) {
        // targetDegrees normalmente va de 0 a 180 para un servo estándar
        return this.runOnce(() -> io.setPivotAngle(targetDegrees));
    }

    // ==========================================================
    // COMANDOS DE FLYWHEEL & SYSID 
    // ==========================================================

    public Command runShooterCommand(double rpmLlanta) {
        return this.run(() -> {
            // 1. GUARDAMOS EL OBJETIVO para que lo vea el dashboard
            objetivoRPMLlanta = rpmLlanta; 

            // 2. Las matemáticas del motor
            double rpmMotor = rpmLlanta / 1.5;
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
            double rpmDeseado = SmartDashboard.getNumber("Shooter/RPM_Test", 0.0);
            
            // B) Guardamos el objetivo para que tu gráfica de AdvantageScope siga funcionando
            objetivoRPMLlanta = rpmDeseado; 

            // C) Las mismas matemáticas perfectas que ya hicimos
            double rpmMotor = rpmDeseado / 1.5;
            double rpsMotor = rpmMotor / 60.0;
            double ffVolts = feedforward.calculate(rpsMotor);
            
            io.setFlywheelVelocity(rpmMotor, ffVolts);
            
        }).finallyDo(() -> {
            objetivoRPMLlanta = 0.0; // Reiniciamos al soltar el botón
            io.stopFlywheel();
        });
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> {
            objetivoRPMLlanta = 0.0; // Reiniciamos el objetivo
            io.stopFlywheel();
            
        });
    }

    public Command activarManual(){
        return this.runEnd(
            ()-> io.setFlywheelVoltage(5),
            () -> io.stopFlywheel()
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}