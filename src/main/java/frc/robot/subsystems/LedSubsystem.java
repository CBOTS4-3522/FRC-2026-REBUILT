package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LedSubsystem extends SubsystemBase {
    
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final PowerDistribution m_pdh;

    // Patrón base: todos los colores, máxima saturación, brillo a la mitad
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // OJO: Asumí 60 LEDs por metro (que es lo estándar en las WS2812B). 
    // Si tu tira es de 30 o 144, cambia este 60.0
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);

    // Animación del arcoíris a 1 metro por segundo
    private final LEDPattern m_scrollingRainbow = 
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    public LedSubsystem() {
        // 1. Inicializar la PDH (Asumiendo que tiene el CAN ID 1, que es el default)
        m_pdh = new PowerDistribution(1, ModuleType.kRev);

        // ENCENDER el puerto conmutable (SW) para darle poder al convertidor
        m_pdh.setSwitchableChannel(true);

        // 2. Configurar la señal de las luces en el puerto PWM 1
        m_led = new AddressableLED(1);

        // 3. Configurar la longitud (40 LEDs)
        m_ledBuffer = new AddressableLEDBuffer(40);
        m_led.setLength(m_ledBuffer.getLength());

        // 4. Iniciar el envío de datos
        m_led.start();
    }

    @Override
    public void periodic() {
        // Actualizar el buffer con la animación calculada en este ciclo
        m_scrollingRainbow.applyTo(m_ledBuffer);
        
        // Mandar los datos físicos a la tira
        m_led.setData(m_ledBuffer);
    }
}