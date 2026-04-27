package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LedSubsystem extends SubsystemBase {

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final PowerDistribution m_pdh;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    private static final Distance kLedSpacing = Meters.of(1 / 60.0);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    // Variable para saber si los mecánicos ya calibraron la máquina
    private boolean isHomed = false;

    // --- ESTADOS DEL OAKAMATIC (LORAX) ---
    public enum EstadoLorax {
        OFF, SUMMER, AUTUMN, WINTER, DISCO
    }

    private EstadoLorax estadoActual = EstadoLorax.OFF;

    public LedSubsystem() {
        m_pdh = new PowerDistribution(1, ModuleType.kRev);
        m_pdh.setSwitchableChannel(true);
        SmartDashboard.putBoolean("Elastic/Modo Lorax", false);
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(38); // Tus 38 LEDs
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
    }

    public void avanzarEstadoLorax() {
        switch (estadoActual) {
            case OFF:
                estadoActual = EstadoLorax.SUMMER;
                break;
            case SUMMER:
                estadoActual = EstadoLorax.AUTUMN;
                break;
            case AUTUMN:
                estadoActual = EstadoLorax.WINTER;
                break;
            case WINTER:
                estadoActual = EstadoLorax.DISCO;
                break;
            case DISCO:
                estadoActual = EstadoLorax.OFF;
                break;
        }
        SmartDashboard.putString("Elastic/Estado Lorax", estadoActual.name());
    }

    public EstadoLorax getEstadoLorax() {
        return estadoActual;
    }

    private void setTodoElColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setHomed(boolean estado) {
        this.isHomed = estado;
    }

    @Override
    public void periodic() {

        if (SmartDashboard.getBoolean("Elastic/Modo Lorax", false)) {
            switch (estadoActual) {
                case OFF:
                    setTodoElColor(0, 0, 0);
                    break;
                case SUMMER:
                    setTodoElColor(0, 255, 0); // Verde
                    break;
                case AUTUMN:
                    setTodoElColor(255, 100, 0); // Naranja
                    break;
                case WINTER:
                    setTodoElColor(200, 200, 255); // Blanco/Azul clarito
                    break;
                case DISCO:
                    m_scrollingRainbow.applyTo(m_ledBuffer); // ¡El arcoíris que ya tenías!
                    break;
            }
            m_led.setData(m_ledBuffer);
            return; // ¡IMPORTANTE! El return hace que ignore el resto del código normal
        }

        if (DriverStation.isTeleopEnabled()) {

            // --- BARRA TEMPORIZADORA DE JUEGO ---
            double matchTime = DriverStation.getMatchTime();
            if (matchTime < 0)
                matchTime = 0;

            var alliance = DriverStation.getAlliance();
            boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

            if (matchTime > 130.0) {
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    if (i % 2 == 0)
                        m_ledBuffer.setRGB(i, 255, 0, 0);
                    else
                        m_ledBuffer.setRGB(i, 0, 0, 255);
                }
            } else if (matchTime > 30.0) {
                double tiempoEnCiclo = (matchTime - 30.0) % 25.0;
                double porcentaje = tiempoEnCiclo / 25.0;
                int ledsEncendidos = (int) (porcentaje * m_ledBuffer.getLength());
                boolean warningBlink = (matchTime % 0.2) > 0.1;

                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    if (i < ledsEncendidos) {
                        if (tiempoEnCiclo <= 4.0) {
                            if (warningBlink)
                                m_ledBuffer.setRGB(i, 255, 120, 0);
                            else
                                m_ledBuffer.setRGB(i, 0, 0, 0);
                        } else {
                            m_ledBuffer.setRGB(i, isRed ? 255 : 0, 0, isRed ? 0 : 255);
                        }
                    } else {
                        m_ledBuffer.setRGB(i, 0, 0, 0);
                    }
                }
            } else {
                boolean blink = (matchTime % 0.5) > 0.25;
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    if (blink)
                        m_ledBuffer.setRGB(i, 255, 60, 0);
                    else
                        m_ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
            m_led.setData(m_ledBuffer);

        } else if (DriverStation.isAutonomousEnabled()) {
            m_scrollingRainbow.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        } else {
            // --- SEMÁFORO DE PITS Y CANCHA (ROBOT APAGADO) ---
            if (!isHomed) {
                // AMARILLO PARPADEANDO: Necesita Homing
                boolean blink = (System.currentTimeMillis() % 1000) < 500;
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setRGB(i, blink ? 255 : 0, blink ? 150 : 0, 0);
                }
            } else {
                // VERDE RESPIRANTE: Homing listo, mecánicos pueden retirarse
                int brightness = (int) (((Math.sin(System.currentTimeMillis() / 300.0) + 1.0) / 2.0) * 80.0);
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setRGB(i, 0, brightness, 0);
                }
            }
            m_led.setData(m_ledBuffer);
        }
    }
}