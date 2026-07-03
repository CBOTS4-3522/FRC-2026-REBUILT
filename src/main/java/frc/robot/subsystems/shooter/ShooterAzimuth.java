/*
 * ShooterAzimuth.java
 * Este es el primer subsistema normal asi que lo voy a explicar a fondo aqui es donde
 * definimos toda la logica y botones y todo lo que se puede hacer con este
 */
package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ShooterAzimuth extends SubsystemBase {
    //traemos la interfaz
    private final ShooterAzimuthIO io;
    //este fue el que creo la libreria y sirve tal cual lo programamos solamente que ya registra las salidas
    private final ShooterAzimuthIOInputsAutoLogged inputs = new ShooterAzimuthIOInputsAutoLogged();
    
    // Controlador de lazo cerrado con perfil trapezoidal
    private final ProfiledPIDController azimuthPID;
    
    //para ver las alertas en la pantalla
    private final Alert alertaEncoderAzimuth = new Alert(
        "¡CRÍTICO! Through Bore Encoder Desconectado (Torreta)", 
        AlertType.kError
    );
    
    private double anguloChamfleManual = 0.0;


    public ShooterAzimuth(ShooterAzimuthIO io) {
        this.io = io;
        //todo lo de smart dashboard es para poderlo ver en la computadora
        SmartDashboard.setDefaultNumber("Shooter/Test_kS_Volts", 0.0);
        
        //un controlador para poderle pedir una pocision especifica al disparador y ademas con velocidad limitada para que no valla de golpe
        azimuthPID = new ProfiledPIDController(
                Constants.shooter.azimuth.kP,
                Constants.shooter.azimuth.kI,
                Constants.shooter.azimuth.kD,
                new TrapezoidProfile.Constraints(
                        Constants.shooter.azimuth.kMaxVelocityDegPerSec,
                        Constants.shooter.azimuth.kMaxAccelerationDegPerSecSq));
                        
        azimuthPID.setTolerance(1.0); // Margen de error aceptable de 1 grado
        //para poderlo modificar dasde la computadora (cambios temporales al reiniciar el robot se vuelve a settear lo que estaba en el constants)
        SmartDashboard.putData("Shooter/Azimuth_PID", azimuthPID);
    }

    public boolean isTorretaEnTope() {
        return inputs.isAzimuthLimitSwitchPressed;
    }
    //esto fue para probar que tanta fricion tenia el shooter y compensar con ello
    /** Comando de diagnóstico para descubrir empíricamente el valor de kS (Fricción estática). */
    public Command probarFriccionEstatica() {
        return this.run(() -> {
            double volts = SmartDashboard.getNumber("Shooter/Test_kS_Volts", 0.0);
            io.setAzimuthVoltage(volts);
        }).finallyDo(() -> {
            io.stopAzimuth();
            SmartDashboard.putNumber("Shooter/Test_kS_Volts", 0.0);
        });
    }
    //esto es lo que se va a repetir una y otra vez no siempre se le pone codigo ya que el robot container va a llamar los comandos que ocupe pero si algo es necesario como registrar cosas se pone aca
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Azimuth", inputs);
        SmartDashboard.putNumber("Shooter/Azimuth_CurrentAngle", inputs.azimuthPositionDegrees);
        alertaEncoderAzimuth.set(inputs.fallaEncoderAbsoluto);
    }

    /** 
     * Control de Posición. Mueve la torreta a un ángulo deseado usando PID y compensación FF. 
     */
    //asi se suele definir los comandos lo que la mayoria ocupa es lamnda aunque
    //hay otra forma larga de definirlos que es por partes al iniciar mientras se ejecuta y al terminar es muy tardado y cada comando se tiene que poner un archivo propio asi que por eso lo hacemos asi
    public Command setAzimuthAngleCommand() {
        return this.run(() -> {
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees);
            double ffOutput = 0.0;
            
            // Inyectar fricción estática (kS) únicamente si el sistema aún no ha llegado a la meta
            if (!azimuthPID.atSetpoint()) {
                ffOutput = Math.signum(pidOutput) * Constants.shooter.azimuth.kS;
            }
            io.setAzimuthVoltage(pidOutput + ffOutput);
        }).finallyDo(() -> io.stopAzimuth());
    }

    /** 
     * Control Teleoperado Manual para Ajuste Fino.
     * Mapea los ejes del control a voltaje de la torreta y ángulo del servo.
     */
    public Command controlManualAzimuth(DoubleSupplier ejeAzimuth, DoubleSupplier ejeChamfle) {
        return this.run(() -> {
            double valAzimuth = MathUtil.applyDeadband(ejeAzimuth.getAsDouble(), 0.1);
            io.setAzimuthVoltage(valAzimuth * 8.0); // Escalado de voltaje para movimiento fino
            
            double valChamfle = MathUtil.applyDeadband(ejeChamfle.getAsDouble(), 0.1);
            anguloChamfleManual += valChamfle * 1.5; // Integración posicional
            anguloChamfleManual = MathUtil.clamp(anguloChamfleManual, 0.0, 180.0);
            
            io.setPivotAngle(anguloChamfleManual);
        }).finallyDo(() -> io.stopAzimuth());
    }

    /** 
     * Comando principal de auto-apuntado inyectado por el cálculo cinemático del Chasis. 
     */
    public Command autoApuntar(DoubleSupplier anguloCalculado) {
        return this.run(() -> {
            // Abrazadera de software (Clamp) para evitar choques estructurales
            double targetSeguro = MathUtil.clamp(anguloCalculado.getAsDouble(), 0.0, 120.0);
            
            double pidOutput = azimuthPID.calculate(inputs.azimuthPositionDegrees, targetSeguro);
            double ffOutput = 0.0;
            if (!azimuthPID.atGoal()) { 
                ffOutput = Math.signum(pidOutput) * Constants.shooter.azimuth.kS;
            }
            
            io.setAzimuthVoltage(pidOutput + ffOutput);
            SmartDashboard.putNumber("AutoAim/Turret_Target", targetSeguro);
        }).finallyDo(() -> io.stopAzimuth());
    }

    public Command resetAzimuthEncoder() {
        return runOnce(() -> io.setAzimuthZero());
    }

    public void setObjetivo(double targetChamfle) {
        io.setPivotAngle(targetChamfle);
    }

    public void detener() {
        io.stopAzimuth();
    }

    /** 
     * Rutina de Homing: Gira lentamente hacia el límite físico para calibrar el 0 absoluto.
     */
    public Command homingCero() {
        return this.run(() -> io.setAzimuthVoltage(-1.5))
                .until(() -> inputs.isAzimuthLimitSwitchPressed)
                .finallyDo(() -> {
                    io.stopAzimuth();
                    io.setAzimuthZero();
                });
    }
}