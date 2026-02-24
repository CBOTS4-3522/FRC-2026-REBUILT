package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    
    @AutoLog
    public static class IntakeRollersIOInputs {
        public double rodillosCorriente = 0.0;
        public double rodillosVoltajeAplicado = 0.0;
        public double rodillosVelocidad = 0.0;

    }

    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    public default void setVoltajeRodillos(double volts) {}
    
    public default void stopRodillos() {}


    
    
}