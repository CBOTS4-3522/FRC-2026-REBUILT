package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double brazoEncoderRaw = 0.0;
        public double brazoCorriente = 0.0;
        public double brazoVoltajeAplicado = 0.0;
        
        public double rodillosVelocidad = 0.0;
        public double rodillosCorriente = 0.0;
        public double rodillosVoltajeAplicado = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
   
    public void setVoltajeRodillos(double volts);

    public void setVoltajeBrazo(double volts);

    public void stopRodillos();
    
    public void stopBrazo();




}

