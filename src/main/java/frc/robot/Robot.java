package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 1. Añade estas importaciones arriba
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends  LoggedRobot{
  private Command m_autonomousCommand;
  RobotContainer m_robotContainer;

   //OBJETOS PARA PRUEBAS (Test Mode)
    private SparkMax testMotor;
    private CommandXboxController testController;

  @Override
  public void robotInit() {
    // Indica el nombre del proyecto
    Logger.recordMetadata("ProjectName", "CBOTS4-2026"); 

    // Guardar logs en la memoria interna de la roboRIO (o USB si hay una)
    Logger.addDataReceiver(new WPILOGWriter()); 

    // Enviar datos por la red para verlos en vivo en AdvantageScope
    Logger.addDataReceiver(new NT4Publisher()); 

    // ¡ESTA ES LA LÍNEA MÁGICA! Sin esto no se guarda nada
    Logger.start(); 

 

    m_robotContainer = new RobotContainer();
    
     //Inicializamos el motor de prueba (ID 50 por ejemplo)
    testMotor = new SparkMax(50, MotorType.kBrushless);
    testController = new CommandXboxController(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    // Publicamos la velocidad del motor de prueba para que los mecánicos la vean
    SmartDashboard.putNumber("Test Motor RPM", testMotor.getEncoder().getVelocity());
  }

  @Override
  public void testInit() {
    // Cancelamos comandos de Teleop para que no interfieran
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // LEER EL GATILLO: El valor va de 0.0 a 1.0
    double speed = testController.getRightTriggerAxis();
    double speedReversed = speed * -1;
    

    // Si el driver presiona el botón 'A', el motor gira en reversa (para sacar piezas)
    if (testController.a().getAsBoolean()) {
        testMotor.set(speedReversed);
    } else {
        testMotor.set(speed);
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  
  
}