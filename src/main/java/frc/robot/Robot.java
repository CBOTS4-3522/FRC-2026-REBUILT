package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// 1. Añade estas importaciones arriba
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends  LoggedRobot{
  private Command m_autonomousCommand;
  RobotContainer m_robotContainer;

  
  @Override
  public void robotInit() {
    // Indica el nombre del proyecto
    Logger.recordMetadata("ProjectName", "CBOTS4-2026"); 

    // Guardar logs en la memoria interna de la roboRIO (o USB si hay una)
    Logger.addDataReceiver(new WPILOGWriter()); 

    // Enviar datos por la red para verlos en vivo en AdvantageScope
    Logger.addDataReceiver(new NT4Publisher()); 

    // ¡esta es la linea magica! Sin esto no se guarda nada
    Logger.start();

 

    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  
  }

  @Override
  public void testInit() {
    // Cancelamos comandos de Teleop para que no interfieran
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  
  
}