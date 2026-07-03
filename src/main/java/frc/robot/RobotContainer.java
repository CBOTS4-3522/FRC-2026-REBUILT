/*
 * RobotContainer.java
 * 
 * Clase principal de la estructura basada en Comandos (Command-Based Programming) de WPILib.
 * Esta clase actúa como el "hub" central del robot donde se instancian los subsistemas,
 * se configuran los dispositivos de entrada (controles) y se enlazan los comandos 
 * a los botones o eventos específicos.
 */

package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Importaciones de subsistemas y abstracciones de Hardware (Interfaces IO)
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.IntakeLift;
import frc.robot.subsystems.intake.IntakeLiftIO;
import frc.robot.subsystems.intake.IntakeLiftOIReal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollersIO;
import frc.robot.subsystems.intake.IntakeRollersIOReal;
import frc.robot.subsystems.shooter.ShooterAzimuth;
import frc.robot.subsystems.shooter.ShooterAzimuthIO;
import frc.robot.subsystems.shooter.ShooterAzimuthIOReal;
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterFlywheelsIO;
import frc.robot.subsystems.shooter.ShooterFlywheelsIOReal;
import frc.robot.subsystems.swerve.SwerveBase;

public class RobotContainer {

        // Interfaz gráfica: Pestaña dedicada a la selección de rutinas autónomas generadas por PathPlanner.
        public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        // Controladores principales (Driver 1: Chasis, Driver 2: Mecanismos).
        private final CommandXboxController driver1 = new CommandXboxController(Constants.OIConstants.kDriver1Port);
        private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

        // Mapas de interpolación (Lookup Tables) para la balística del robot.
        // Asignan la Distancia al objetivo (X) con los valores óptimos de RPM, Tiempo de vuelo y Ángulo (Y).
        private final InterpolatingDoubleTreeMap mapaRPM = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaTiempo = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaChamfle = new InterpolatingDoubleTreeMap();

        // -------------------------------------------------------------------
        // DECLARACIÓN DE SUBSISTEMAS
        // -------------------------------------------------------------------
        
        // Subsistema de visión basado en AprilTags para estimación de pose.
        private final Vision s_Vision;
        
        // Subsistema de tracción holonómica (Swerve Drive) para el desplazamiento.
        private final SwerveBase s_Swerve;
        
        /* 
         * Nota de Arquitectura: El Shooter está dividido en dos subsistemas distintos (Azimuth y Flywheels).
         * WPILib tiene un requerimiento estricto de concurrencia: solo un Command puede requerir 
         * un subsistema a la vez. Al separarlos, podemos tener un comando rotando la torreta (Azimuth) 
         * de manera simultánea e independiente al comando que acelera las ruedas (Flywheels).
         */
        private final ShooterAzimuth s_ShooterAzimuth;       // Control rotacional de la torreta.
        private final ShooterFlywheels s_ShooterFlywheels;   // Control de velocidad de disparo (RPM).
        
        private final Indexer s_Indexer;                     // Bandas transportadoras internas.
        private final IntakeLift s_IntakeLift;               // Actuador lineal/brazo para bajar/subir la recolección.
        private final IntakeRollers s_IntakeRollers;         // Rodillos de ingesta de piezas.
        private final LedSubsystem s_LedSubsystem;           // Feedback visual de estado para los drivers.

        // Selector dinámico para la rutina autónoma enviado al Dashboard.
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                // Inicialización de subsistemas sin dependencias complejas de IO.
                s_Swerve = new SwerveBase();
                s_LedSubsystem = new LedSubsystem();
                s_Vision = new Vision(s_Swerve); // Inyectamos el chasis para actualizar la odometría con la visión.

                // -------------------------------------------------------------------
                // POBLACIÓN DE MAPAS DE INTERPOLACIÓN (Balística)
                // -------------------------------------------------------------------
                // Formato: .put(Distancia en metros, Valor deseado)
                mapaRPM.put(1.5, 2600.0);
                mapaChamfle.put(1.5, 0.0);
                mapaTiempo.put(1.5, 0.87);
                mapaRPM.put(2.0, 2650.0);
                mapaChamfle.put(2.0, 20.0);
                mapaTiempo.put(2.0, 0.9566);
                mapaRPM.put(2.5, 2750.0);
                mapaChamfle.put(2.5, 40.0);
                mapaTiempo.put(2.5, 1.0333);
                mapaRPM.put(3.0, 2950.0);
                mapaChamfle.put(3.0, 70.0);
                mapaTiempo.put(3.0, 1.063);
                mapaRPM.put(3.5, 3100.0);
                mapaChamfle.put(3.5, 90.0);
                mapaTiempo.put(3.5, 1.18);
                mapaRPM.put(4.0, 3300.0);
                mapaChamfle.put(4.0, 120.0);
                mapaTiempo.put(4.0, 1.19);
                mapaRPM.put(4.5, 3400.0);
                mapaChamfle.put(4.5, 140.0);
                mapaTiempo.put(4.5, 1.22);
                mapaRPM.put(5.0, 3500.0);
                mapaChamfle.put(5.0, 160.0);
                mapaTiempo.put(5.0, 1.25);
                mapaRPM.put(5.5, 3650.0);
                mapaChamfle.put(5.5, 170.0);
                mapaTiempo.put(5.5, 1.36);
                mapaRPM.put(6.0, 3850.0);
                mapaChamfle.put(6.0, 180.0);
                mapaTiempo.put(6.0, 1.45);

                // -------------------------------------------------------------------
                // INYECCIÓN DE DEPENDENCIAS Y PATRÓN IO (HARDWARE ABSTRACTION)
                // -------------------------------------------------------------------
                /* 
                 * Si el código corre en la RoboRIO (hardware real), instanciamos las clases *IOReal
                 * que interactúan con CAN, SparkMax, etc. Si corre en el simulador, pasamos una 
                 * interfaz vacía/mockeada para evitar excepciones de hardware no encontrado y poder 
                 * simular la lógica y la física (AdvantageKit framework format).
                 */
                if (RobotBase.isReal()) {
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIOReal());
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOReal());
                        s_Indexer = new Indexer(new IndexerIOReal());
                        s_IntakeLift = new IntakeLift(new IntakeLiftOIReal());
                        s_IntakeRollers = new IntakeRollers(new IntakeRollersIOReal());
                } else {
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIO() {});
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIO() {});
                        s_Indexer = new Indexer(new IndexerIO() {});
                        s_IntakeLift = new IntakeLift(new IntakeLiftIO() {});
                        s_IntakeRollers = new IntakeRollers(new IntakeRollersIO() {});
                }

                // -------------------------------------------------------------------
                // REGISTRO DE COMANDOS PARA PATHPLANNER (Fase Autónoma)
                // -------------------------------------------------------------------
                /*
                 * Vinculación de strings definidos en la UI de PathPlanner con comandos instanciables 
                 * de Java. Esto permite que las trayectorias autónomas ejecuten acciones complejas.
                 */
                NamedCommands.registerCommand("DISPARAR_SMART",
                        // Commands.parallel: Ejecuta de forma concurrente, siempre que no compartan subsistema.
                        Commands.parallel(
                                s_ShooterFlywheels.runShooterCommand(2650),
                                
                                // Commands.sequence: Ejecuta en orden secuencial.
                                Commands.sequence(
                                        // Bloqueo hasta alcanzar setpoint de RPM
                                        Commands.waitUntil(s_ShooterFlywheels::estaEnVelocidad),
                                        
                                        // Inicia alimentación al shooter con sistema anti-atasco integrado
                                        Commands.parallel(
                                                s_Indexer.dispararConAntiAtasco(s_ShooterFlywheels::detectoBajonPelota),
                                                s_IntakeRollers.movimiento()
                                        )
                                )
                        )
                );
                
                NamedCommands.registerCommand("TRAGAR", s_IntakeRollers.tragarPelotas());
                NamedCommands.registerCommand("BAJAR_INTAKE", s_IntakeLift.bajarProtegido());
                NamedCommands.registerCommand("PRENDER_INDEXER", s_Indexer.encender());
                NamedCommands.registerCommand("MENEITO", s_Swerve.sacudirChasis()); // Oscilación sinusoidal para desatascar.
                NamedCommands.registerCommand("DISPARAR_AUTO_AIM", disparoInteligente());
                NamedCommands.registerCommand("HOMING SHOOTER", s_ShooterAzimuth.homingCero());

                // -------------------------------------------------------------------
                // PESTAÑA DE DIAGNÓSTICO (SYSID & GYRO)
                // -------------------------------------------------------------------
                // Herramientas de telemetría y rutinas de System Identification (SysId) 
                // para calcular constantes de FeedForward (kS, kV, kA) del chasis y shooter.
                ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");

                diagTab.add("Quasistatic Forward", new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kForward), Set.of(s_Swerve))).withSize(2, 1).withPosition(0, 0);
                diagTab.add("Quasistatic Reverse", new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kReverse), Set.of(s_Swerve))).withSize(2, 1).withPosition(2, 0);
                diagTab.add("Dynamic Forward", new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kForward), Set.of(s_Swerve))).withSize(2, 1).withPosition(0, 1);
                diagTab.add("Dynamic Reverse", new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kReverse), Set.of(s_Swerve))).withSize(2, 1).withPosition(2, 1);
                diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro).withSize(2, 2).withPosition(4, 0);

                diagTab.add("Shooter QS Fwd", new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kForward), Set.of(s_ShooterFlywheels))).withSize(2, 1).withPosition(0, 2);
                diagTab.add("Shooter QS Rev", new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kReverse), Set.of(s_ShooterFlywheels))).withSize(2, 1).withPosition(2, 2);
                diagTab.add("Shooter Dyn Fwd", new DeferredCommand(() -> s_ShooterFlywheels.sysIdDynamic(Direction.kForward), Set.of(s_ShooterFlywheels))).withSize(2, 1).withPosition(0, 3);
                diagTab.add("Shooter Dyn Rev", new DeferredCommand(() -> s_ShooterFlywheels.sysIdDynamic(Direction.kReverse), Set.of(s_ShooterFlywheels))).withSize(2, 1).withPosition(2, 3);
                SmartDashboard.putData("Shooter/Activar Test kS", s_ShooterAzimuth.probarFriccionEstatica());

                // -------------------------------------------------------------------
                // CONFIGURACIONES POR DEFECTO Y SELECTOR AUTÓNOMO
                // -------------------------------------------------------------------
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Selector", autoChooser);
                
                // Configuración de tracción predeterminada (Teleoperado) ligada a los ejes del Driver 1
                s_Swerve.setDefaultCommand(new TeleopSwerve(
                                s_Swerve,
                                () -> -driver1.getLeftY(),
                                () -> -driver1.getLeftX(),
                                () -> driver1.getRightX(),
                                () -> driver1.getLeftTriggerAxis(),
                                () -> driver1.getHID().getLeftBumperButton(),
                                () -> driver1.getHID().getRightBumperButton(),
                                () -> driver1.getHID().getPOV()));

                // Configuración de movimiento manual de torreta ligado al Driver 2
                s_ShooterAzimuth.setDefaultCommand(s_ShooterAzimuth.controlManualAzimuth(
                                () -> -driver2.getLeftX(),
                                () -> -driver2.getLeftY()));

                // Enlace de botones del control a los comandos (Action mapping)
                configureButtonBindings();
        }

        /**
         * Rutina de seguridad ejecutada antes de los partidos.
         * Asegura la correcta calibración posicional (Zeroing) del encoder de la torreta.
         */
        public Command rutinaPreMatchManual() {
                return Commands.either(
                                Commands.sequence(
                                                s_ShooterAzimuth.resetAzimuthEncoder(),
                                                Commands.runOnce(() -> s_LedSubsystem.setHomed(true))),
                                Commands.none(), 
                                s_ShooterAzimuth::isTorretaEnTope // Condición evaluada
                ).ignoringDisable(true); // Se permite su ejecución estando el robot en estado Disabled
        }

        /**
         * Comando para la alineación evasiva del chasis.
         * Mantiene el cálculo cinemático de rotación hacia el objetivo (Targeting) 
         * utilizando un controlador PID, permitiendo al mismo tiempo el movimiento 
         * translacional evasivo gestionado por el conductor.
         */
        public Command alinearChasisEvasivo(DoubleSupplier translationX, DoubleSupplier translationY) {
                return Commands.run(() -> {
                        Pose2d robotPose = s_Swerve.getPose();
                        Rotation2d robotYaw = s_Swerve.getYaw();
                        
                        // Cinemática inversa para obtener ángulo directo hacia el objetivo
                        Rotation2d anguloHaciaMeta = Rotation2d.fromRadians(
                                        Math.atan2(
                                                        Constants.shooter.getObjetivoActual().getY() - robotPose.getY(),
                                                        Constants.shooter.getObjetivoActual().getX() - robotPose.getX()));

                        // ESTRATEGIA DE OFFSET: Sumamos 30 grados para descentrar el chasis.
                        // Esto obliga a la torreta a compensar asimétricamente, garantizando 
                        // margen de maniobra en ambos sentidos si el piloto debe esquivar bruscamente.
                        Rotation2d anguloChasisDeseado = anguloHaciaMeta.plus(Rotation2d.fromDegrees(30.0));
                        
                        // Filtrado de deadband de los joysticks
                        double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.OIConstants.kStickDeadband);
                        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.OIConstants.kStickDeadband);

                        // Cálculo del control en lazo cerrado para la orientación del chasis
                        double rotacionPID = s_Swerve.headingController.calculate(robotYaw.getDegrees(), anguloChasisDeseado.getDegrees());
                        double rotacionLimitada = MathUtil.clamp(rotacionPID, -1.0, 1.0);
                        
                        s_Swerve.drive(
                                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.kMaxSpeed),
                                        rotacionLimitada * Constants.Swerve.kMaxAngularVelocity,
                                        true, false);
                                        
                        SmartDashboard.putNumber("AutoAim/AnguloChasis_Deseado", anguloChasisDeseado.getDegrees());
                }, s_Swerve);
        }

        /**
         * Computa la solución de disparo dinámico ("Shoot-on-the-move").
         * Modifica el objetivo estático basándose en el vector de velocidad actual del chasis,
         * creando una meta virtual iterativa.
         */
        public Command disparoInteligente() {
                return Commands.parallel(
                                // ======================================================
                                // 1. APUNTADO PREDICTIVO (Azimuth & Hood)
                                // ======================================================
                                s_ShooterAzimuth.autoApuntar(() -> {
                                        Pose2d robotPose = s_Swerve.getPose();
                                        Rotation2d robotYaw = s_Swerve.getYaw();

                                        // Transformación geométrica para situar el origen del disparo real
                                        Translation2d posicionShooter = robotPose.getTranslation().plus(
                                                        Constants.shooter.kShooterOffset.rotateBy(robotYaw));

                                        var velocidades = s_Swerve.getRobotRelativeSpeeds();
                                        // Rotación de las velocidades para mapearlas al campo
                                        double vxField = velocidades.vxMetersPerSecond * robotYaw.getCos()
                                                        - velocidades.vyMetersPerSecond * robotYaw.getSin();
                                        double vyField = velocidades.vxMetersPerSecond * robotYaw.getSin()
                                                        + velocidades.vyMetersPerSecond * robotYaw.getCos();

                                        double distanciaReal = posicionShooter.getDistance(Constants.shooter.getObjetivoActual());
                                        double tiempoVuelo = mapaTiempo.get(distanciaReal);

                                        // Creación del Objetivo Virtual (Proyección cinemática bidimensional)
                                        double metaVirtualX = Constants.shooter.getObjetivoActual().getX() - (vxField * tiempoVuelo);
                                        double metaVirtualY = Constants.shooter.getObjetivoActual().getY() - (vyField * tiempoVuelo);

                                        Rotation2d anguloHaciaMetaVirtual = Rotation2d.fromRadians(
                                                        Math.atan2(metaVirtualY - posicionShooter.getY(),
                                                                        metaVirtualX - posicionShooter.getX()));

                                        double distanciaVirtual = Math.hypot(metaVirtualX - posicionShooter.getX(), metaVirtualY - posicionShooter.getY());
                                        
                                        // Ajuste del ángulo de hood/chamfle
                                        s_ShooterAzimuth.setObjetivo(mapaChamfle.get(distanciaVirtual));

                                        return anguloHaciaMetaVirtual.minus(robotYaw).getDegrees() + 85.0;
                                }),

                                // ======================================================
                                // 2. CONTROL RPM DINÁMICO
                                // ======================================================
                                Commands.run(() -> {
                                        Pose2d robotPose = s_Swerve.getPose();
                                        Translation2d posicionShooter = robotPose.getTranslation().plus(
                                                        Constants.shooter.kShooterOffset.rotateBy(robotPose.getRotation()));

                                        double distanciaReal = posicionShooter.getDistance(Constants.shooter.getObjetivoActual());
                                        double rpmDeseado = mapaRPM.get(distanciaReal);

                                        SmartDashboard.putNumber("AutoAim/Dist_CentroRobot", robotPose.getTranslation().getDistance(Constants.shooter.getObjetivoActual()));
                                        SmartDashboard.putNumber("AutoAim/Dist_ShooterReal", distanciaReal);
                                        SmartDashboard.putNumber("AutoAim/RPM_Mapeado", rpmDeseado);

                                        s_ShooterFlywheels.setObjetivoRPM(rpmDeseado);
                                }, s_ShooterFlywheels),

                                // ======================================================
                                // 3. SECUENCIA LÓGICA DE FUEGO (Indexer)
                                // ======================================================
                                Commands.sequence(
                                                Commands.waitUntil(s_ShooterFlywheels::estaEnVelocidad),
                                                s_Indexer.dispararConAntiAtasco(s_ShooterFlywheels::detectoBajonPelota)
                                )
                                // Finalizador de seguridad para detener motores
                                .finallyDo(() -> {
                                        s_ShooterFlywheels.detener();
                                        s_ShooterAzimuth.detener();
                                }));
        }

        /**
         * Manejador de feedback háptico (Vibración) para el conductor.
         */
        public Command vibrarDriver(CommandXboxController driverM, XboxController.RumbleType tipo, double magnitud, double tiempo) {
                return Commands.runEnd(
                                () -> driverM.getHID().setRumble(tipo, magnitud),
                                () -> driverM.getHID().setRumble(tipo, 0)).withTimeout(tiempo);
        }

        /**
         * Asignación de botones de los gamepads a comandos específicos.
         * Implementa el patrón Observer interno de WPILib2.
         */
        private void configureButtonBindings() {
                // Reinicio del giroscopio a 0 (Field-Oriented reset)
                driver1.rightStick().onTrue(new InstantCommand(s_Swerve::zeroGyro));
                
                // Ajuste de odometría inicial basado en alianza (Field Symmetry handling)
                driver1.start().onTrue(
                                Commands.runOnce(() -> {
                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                                                s_Swerve.resetOdometry(new Pose2d(12.97, 4.0, Rotation2d.fromDegrees(0)));
                                        } else {
                                                s_Swerve.resetOdometry(new Pose2d(3.571, 4.0, Rotation2d.fromDegrees(180)));
                                        }
                                }));
                
                driver1.leftStick().whileTrue(s_Swerve.sacudirChasis());

                // ==========================================================
                // DRIVER 1: INTERACCIÓN CHASIS
                // ==========================================================
                // Orientación auto-evasiva con gatillo
                driver1.leftTrigger().whileTrue(
                                alinearChasisEvasivo(
                                                () -> -driver1.getLeftY(),
                                                () -> -driver1.getLeftX()));
                
                // Formación en "X" de las ruedas para evitar empujes defensivos
                driver1.x().whileTrue(Commands.run(s_Swerve::wheelsIn, s_Swerve));
                
                // On-The-Fly PathPlanner execution
                driver1.y().whileTrue(new PathPlannerAuto("regresar_a_disparar_BLUE"));                

                // ==========================================================
                // DRIVER 2: OPERADOR DE MECANISMOS
                // ==========================================================
                driver2.x().toggleOnTrue(s_IntakeRollers.tragarPelotas());
                driver2.leftBumper().whileTrue(s_Indexer.encender());
                driver2.leftTrigger().whileTrue(s_Indexer.alRevez()); // Reversa
                
                // Manipulación del Brazo
                driver2.a().onTrue(s_IntakeLift.bajarProtegido());
                driver2.b().onTrue(s_IntakeLift.subirProtegido());
                
                // Testeo y Secuencias manuales de disparo
                driver2.rightStick().whileTrue(s_ShooterFlywheels.testShooterDesdeDashboard());
                driver2.y().whileTrue(
                                Commands.parallel(
                                        s_ShooterFlywheels.testShooterDesdeDashboard(), 
                                        Commands.sequence(
                                                Commands.waitUntil(s_ShooterFlywheels::estaEnVelocidad),
                                                s_Indexer.dispararConAntiAtasco(s_ShooterFlywheels::detectoBajonPelota))));

                driver2.rightBumper().whileTrue(s_IntakeRollers.movimiento());
                
                // Control D-PAD de Torreta (Homing y Ajustes)
                driver2.povRight().onTrue(s_ShooterAzimuth.homingCero());
                driver2.povDown().onTrue(rutinaPreMatchManual());
                driver2.povUp().toggleOnTrue(s_ShooterAzimuth.setAzimuthAngleCommand());

                // Fuego inteligente automático en movimiento.
                driver2.rightTrigger().whileTrue(disparoInteligente());
        }

        /**
         * Retorna la rutina autónoma seleccionada por el equipo en el Dashboard.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}