/*
 * ShooterAzimuthIO.java
 *
 * Aqui se definen las entradas y funciones que van a actuar con el codigo, lo que se define aqui el programa
 * lo registra automaticamente y al verlo desde la computadora o en la usb podemos ver estos datos en vivo
 */
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterAzimuthIO {
    //esto es algo raro porque la misma funcion crea otro archivo automaticamente en el codigo asi que esto de aqui como tal no se manda llamar
    //lo que se manda llamar es la que crea la libreria
    @AutoLog
    public static class ShooterAzimuthIOInputs {
        // --- Azimuth (Torreta - Spark MAX) ---
        public double azimuthPositionDegrees = 0.0;
        public double azimuthVelocityDegPerSec = 0.0;
        public double azimuthAppliedVolts = 0.0;
        public double azimuthCurrentAmps = 0.0;
        public boolean isAzimuthLimitSwitchPressed = false;
        
        // --- Pivot (Ángulo de Tiro / Servo PWM) ---
        public double pivotAngleDegrees = 0.0;
        public boolean fallaEncoderAbsoluto = false;
    }
    //y las funciones para accionar las cosas
    public default void updateInputs(ShooterAzimuthIOInputs inputs) {}
    public default void setAzimuthVoltage(double volts) {}
    public default void stopAzimuth() {}
    public default void setAzimuthPosition(double degrees) {}
    public default void setAzimuthZero() {}
    public default void setPivotAngle(double degrees) {}
}