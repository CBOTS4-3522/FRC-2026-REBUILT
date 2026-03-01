package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAzimuth extends SubsystemBase {

    private final ShooterAzimuthIO io;
    private final ShooterAzimuthIOInputsAutoLogged inputs = new ShooterAzimuthIOInputsAutoLogged();


    private final ProfiledPIDController azimuthPID;
  

    

   
    private double anguloChamfleManual = 0.0;

    public ShooterAzimuth(ShooterAzimuthIO io) {
        this.io = io;

     

   
        azimuthPID = new ProfiledPIDController(
                0.015, 0.0, 0.0, 
                new TrapezoidProfile.Constraints(180, 100) 
        );
        azimuthPID.setTolerance(1.0); 

        SmartDashboard.putData("Shooter/Azimuth_PID", azimuthPID);

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

    public Command setAzimuthAngleCommand(double targetDegrees) {
        return this.run(() -> {
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees, targetDegrees);
            io.setAzimuthVoltage(pidOutput);
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
            
            // Si mueven la palanca, le sumamos grados a la memoria (ej. 1.5 grados cada 20ms)
            anguloChamfleManual += valChamfle * 1.5; 
            
            // Límite duro de software para no tronar el servo mecánicamente
            anguloChamfleManual = MathUtil.clamp(anguloChamfleManual, 0.0, 180.0);
            
            io.setPivotAngle(anguloChamfleManual);
            
        }).finallyDo(() -> {
            io.stopAzimuth(); // Apagamos el motor de la torreta por seguridad
            // OJO: El servo NO se apaga. Mantiene el último 'anguloChamfleManual' que calculó.
        });
    }

    public Command resetAzimuthEncoder() {
        return runOnce(() -> io.setAzimuthZero());
    }

    public Command manualAzimuthCommand(double volts) {
        return this.runEnd(() -> io.setAzimuthVoltage(volts), ()-> io.stopAzimuth());
    }

    

    // ==========================================================
    // COMANDOS DE PIVOT (Chamfle con Servo)
    // ==========================================================

    public Command setPivotAngleCommand(double targetDegrees) {
        // targetDegrees normalmente va de 0 a 180 para un servo estándar
        return this.runOnce(() -> io.setPivotAngle(targetDegrees));
    }

 
}