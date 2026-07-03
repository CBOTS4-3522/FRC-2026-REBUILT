/*
 * Robot.java
 * 
 * Clase principal que actúa como el punto de entrada de la Máquina Virtual de Java (JVM)
 * del robot. Hereda de LoggedRobot (AdvantageKit framework) para habilitar el registro 
 * avanzado de telemetría, lo que permite grabar los datos en la RoboRIO y visualizarlos 
 * o hacer un "replay" en AdvantageScope.
 * 
 * Además, gestiona la máquina de estados de WPILib (Disabled, Autonomous, Teleop, Test).
 */

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Importaciones de AdvantageKit para telemetría y logging
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  
  // Comando que almacenará la rutina autónoma seleccionada
  private Command m_autonomousCommand;
  
  // Contenedor principal donde residen los subsistemas y la configuración de botones
  private RobotContainer m_robotContainer;

  /**
   * robotInit()
   * Se ejecuta una única vez cuando el robot se enciende o el código se reinicia.
   * Es el lugar ideal para inicializar sistemas globales, la telemetría y el RobotContainer.
   */
  @Override
  public void robotInit() {
    // Configuración de metadatos para AdvantageKit (Identificación del log)
    Logger.recordMetadata("ProjectName", "CBOTS4-2026"); 

    // Configura la escritura física de los logs en la memoria flash de la RoboRIO o USB conectada[cite: 1]
    Logger.addDataReceiver(new WPILOGWriter()); 

    // Publica los datos a través de NetworkTables (NT4) para visualización en vivo en AdvantageScope[cite: 1]
    Logger.addDataReceiver(new NT4Publisher()); 
    
    // Inicia el hilo de registro de telemetría[cite: 1]
    Logger.start();

    // Instancia el contenedor principal, inicializando subsistemas y dependencias de hardware[cite: 1]
    m_robotContainer = new RobotContainer();
  }

  /**
   * robotPeriodic()
   * Bucle principal del robot. Se ejecuta cíclicamente cada 20 ms independientemente del modo.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Ejecuta el CommandScheduler. Esto es responsable de hacer un sondeo (polling) 
     * a los botones, ejecutar los comandos programados y actualizar los subsistemas.
     * Si esto se elimina, el robot basado en comandos dejará de funcionar.
     */
    CommandScheduler.getInstance().run();
  }

  /**
   * autonomousInit()
   * Se ejecuta una única vez al inicio del periodo Autónomo (los primeros 15 segundos del match).
   */
  @Override
  public void autonomousInit() {
    // Recupera la rutina seleccionada en el Dashboard (PathPlanner auto)[cite: 1]
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Programa la ejecución de la rutina si existe[cite: 1]
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * teleopInit()
   * Se ejecuta una única vez al inicio del periodo Teleoperado (control manual).
   */
  @Override
  public void teleopInit() {
    // Cancelamos la rutina autónoma para evitar que siga moviendo el robot
    // cuando el piloto (humano) toma el control[cite: 1]
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * testInit()
   * Se ejecuta una única vez al entrar al modo de Pruebas (Test Mode).
   * Útil para calibración de mecanismos o diagnóstico en pits sin activar comandos de partida.
   */
  @Override
  public void testInit() {
    // Cancela todos los comandos en ejecución para asegurar un estado limpio en pruebas[cite: 1]
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * testPeriodic()
   * Bucle que se ejecuta cada 20 ms durante el modo de Pruebas.
   */
  @Override
  public void testPeriodic() {
    // Lógica específica para modo test (por ejemplo, validación de motores)
  }
}