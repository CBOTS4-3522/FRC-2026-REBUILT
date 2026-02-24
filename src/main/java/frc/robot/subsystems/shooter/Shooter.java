package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SysIdRoutine m_sysIdRoutine;
    private final ProfiledPIDController azimuthPID;
    private double objetivo = 3000/1.5;

    public Shooter(ShooterIO io) {
        this.io = io;

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        SmartDashboard.putNumber("Shooter/Azimuth_CurrentAngle", inputs.azimuthPositionDegrees);
        SmartDashboard.putNumber("Shooter/RPMObjetivo", objetivo);
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

    public Command runShooterCommand(double rpm) {
        return this.run(() -> io.setFlywheelVelocity(rpm/1.5));
    }

    public Command stopShooterCommand() {
        return this.runOnce(() -> io.stopFlywheel());
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