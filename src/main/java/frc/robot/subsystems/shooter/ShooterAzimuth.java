package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterAzimuth extends SubsystemBase {

    private final ShooterAzimuthIO io;
    private final ShooterAzimuthIOInputsAutoLogged inputs = new ShooterAzimuthIOInputsAutoLogged();

    private final ProfiledPIDController azimuthPID;

    // private final SimpleMotorFeedforward feedforward = new
    // SimpleMotorFeedforward(
    // Constants.shooter.azimuth.kS,
    // Constants.shooter.azimuth.kV
    // );

    private double anguloChamfleManual = 0.0;

    public ShooterAzimuth(ShooterAzimuthIO io) {
        this.io = io;
        SmartDashboard.setDefaultNumber("Shooter/Test_kS_Volts", 0.0);  
        azimuthPID = new ProfiledPIDController(
                Constants.shooter.azimuth.kP,
                Constants.shooter.azimuth.kI,
                Constants.shooter.azimuth.kD,
                new TrapezoidProfile.Constraints(
                        Constants.shooter.azimuth.kMaxVelocityDegPerSec,
                        Constants.shooter.azimuth.kMaxAccelerationDegPerSecSq));
        azimuthPID.setTolerance(1.0);
        azimuthPID.setTolerance(1.0);

        SmartDashboard.putData("Shooter/Azimuth_PID", azimuthPID);

    }

    public boolean isTorretaEnTope() {
        return inputs.isAzimuthLimitSwitchPressed;
    }

    public Command probarFriccionEstatica() {
        return this.run(() -> {
            // Leemos el voltaje que escribiste en la pantalla
            double volts = SmartDashboard.getNumber("Shooter/Test_kS_Volts", 0.0);
            
            // Se lo inyectamos directo al motor
            io.setAzimuthVoltage(volts);
        })
        .finallyDo(() -> {
            // Por seguridad, apagamos el motor y regresamos el valor a 0 en la pantalla al cancelar el comando
            io.stopAzimuth();
            SmartDashboard.putNumber("Shooter/Test_kS_Volts", 0.0);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Azimuth", inputs);

        SmartDashboard.putNumber("Shooter/Azimuth_CurrentAngle", inputs.azimuthPositionDegrees);

    }

    // ==========================================================
    // COMANDOS DE AZIMUTH (Torreta)
    // ==========================================================

    public Command setAzimuthAngleCommand() {
        return this.run(() -> {
            // 1. PID puro: Da todo el voltaje posible basado en qué tan lejos está
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees);

            // 2. Fricción estática (kS): Solo si estamos fuera de tolerancia, inyectamos
            // ese empujoncito
            double ffOutput = 0.0;
            if (!azimuthPID.atSetpoint()) {
                ffOutput = Math.signum(pidOutput) * Constants.shooter.azimuth.kS;
            }

            // 3. ¡Todo el poder al motor!
            io.setAzimuthVoltage(pidOutput + ffOutput);

        })
                .finallyDo(() -> io.stopAzimuth());
    }

    public Command controlManualAzimuth(DoubleSupplier ejeAzimuth, DoubleSupplier ejeChamfle) {
        return this.run(() -> {
            // ==========================================
            // 1. AZIMUTH (Torreta) - Control de Velocidad
            // ==========================================
            double valAzimuth = MathUtil.applyDeadband(ejeAzimuth.getAsDouble(), 0.1);
            // Multiplicamos por 4.0V máximo para que se mueva muy "de a poquito"
            io.setAzimuthVoltage(valAzimuth * 8.0);

            // ==========================================
            // 2. CHAMFLE (Servo) - Control de Posición
            // ==========================================
            double valChamfle = MathUtil.applyDeadband(ejeChamfle.getAsDouble(), 0.1);

            // Si mueven la palanca, le sumamos grados a la memoria (ej. 1.5 grados cada
            // 20ms)
            anguloChamfleManual += valChamfle * 1.5;

            // Límite duro de software para no tronar el servo mecánicamente
            anguloChamfleManual = MathUtil.clamp(anguloChamfleManual, 0.0, 180.0);

            io.setPivotAngle(anguloChamfleManual);

        }).finallyDo(() -> {
            io.stopAzimuth(); // Apagamos el motor de la torreta por seguridad
            // OJO: El servo NO se apaga. Mantiene el último 'anguloChamfleManual' que
            // calculó.
        });
    }

    public Command autoApuntar(DoubleSupplier anguloCalculado) {
    return this.run(() -> {
        // 1. Leemos la matemática en vivo y limitamos al rango seguro (0 a 120)
        double targetSeguro = MathUtil.clamp(anguloCalculado.getAsDouble(), 0.0, 120.0);
        
        // 2. Calculamos el PID Perfilado (Aceleración y velocidad controladas)
        double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees, targetSeguro);
        
        // 3. Fricción estática (kS) inyectada solo si no hemos llegado a la meta
        double ffOutput = 0.0;
        if (!azimuthPID.atGoal()) { 
            ffOutput = Math.signum(pidOutput) * Constants.shooter.azimuth.kS;
        }
        
        // 4. Mover suavemente
        io.setAzimuthVoltage(pidOutput + ffOutput);
        SmartDashboard.putNumber("AutoAim/Turret_Target", targetSeguro);
        
    }).finallyDo(() -> io.stopAzimuth());
}

    public Command resetAzimuthEncoder() {
        return runOnce(() -> io.setAzimuthZero());
    }

    public Command manualAzimuthCommand(double volts) {
        return this.runEnd(() -> io.setAzimuthVoltage(volts), () -> io.stopAzimuth());
    }

    public void setObjetivo(double targetChamfle) {

        io.setPivotAngle(targetChamfle);
    }

    public void detener() {
        io.stopAzimuth();
    }

    // ==========================================================
    // COMANDOS DE PIVOT (Chamfle con Servo)
    // ==========================================================

    public Command setPivotAngleCommand(double targetDegrees) {
        // targetDegrees normalmente va de 0 a 180 para un servo estándar
        return this.runOnce(() -> io.setPivotAngle(targetDegrees));
    }

    public Command homingCero() {
        return this.run(() -> {
            // Giramos hacia la derecha (CW / Negativo) para buscar el 0
            io.setAzimuthVoltage(-1.5);
        })
                .until(() -> inputs.isAzimuthLimitSwitchPressed) // Se detiene al tocar el límite de reversa
                .finallyDo(() -> {
                    io.stopAzimuth();

                    io.setAzimuthZero();
                });
    }

}