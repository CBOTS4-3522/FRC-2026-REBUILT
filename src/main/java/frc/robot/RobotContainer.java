package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.robot.commands.TeleopSwerve;
<<<<<<< HEAD
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
=======
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Vision;
>>>>>>> feature/vision
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
        public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        private final CommandXboxController driver1 = new CommandXboxController(Constants.OIConstants.kDriver1Port);
        private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

<<<<<<< HEAD
        private final InterpolatingDoubleTreeMap mapaRPM = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaTiempo = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaChamfle = new InterpolatingDoubleTreeMap();

=======
        private final Vision s_Vision;

        /* Subsystems */
>>>>>>> feature/vision
        private final SwerveBase s_Swerve;
        private final ShooterAzimuth s_ShooterAzimuth;
        private final ShooterFlywheels s_ShooterFlywheels;
        private final Indexer s_Indexer;
        private final IntakeLift s_IntakeLift;
        private final IntakeRollers s_IntakeRollers;
     

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                s_Swerve = new SwerveBase();

<<<<<<< HEAD
                // -------------------------------------------------------------------
                // MAPAS DE INTERPOLACIÓN
                // -------------------------------------------------------------------
                mapaRPM.put(1.5, 2600.0); mapaTiempo.put(1.5, 1.20); mapaChamfle.put(1.5, 0.0);
                mapaRPM.put(2.0, 2800.0); mapaTiempo.put(2.0, 1.00); mapaChamfle.put(2.0, 0.0);
                mapaRPM.put(2.5, 3000.0); mapaTiempo.put(2.5, 1.48); mapaChamfle.put(2.5, 60.0);
                mapaRPM.put(3.0, 3200.0); mapaTiempo.put(3.0, 1.35); mapaChamfle.put(3.0, 80.0);
                mapaRPM.put(3.5, 3400.0); mapaTiempo.put(3.5, 1.45); mapaChamfle.put(3.5, 100.0);
                mapaRPM.put(4.0, 3600.0); mapaTiempo.put(4.0, 1.58); mapaChamfle.put(4.0, 120.0);
                mapaRPM.put(4.5, 3800.0); mapaTiempo.put(4.5, 1.75); mapaChamfle.put(4.5, 140.0);
=======
                s_Vision = new Vision(s_Swerve);

                // ShooterIO shooterIO; // 1. Declaramos la interfaz temporal
>>>>>>> feature/vision

                // -------------------------------------------------------------------
                // INICIALIZACIÓN DE SUBSISTEMAS (REAL VS SIMULADO)
                // -------------------------------------------------------------------
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
                // REGISTRO DE COMANDOS PATHPLANNER
                // -------------------------------------------------------------------
                NamedCommands.registerCommand("DISPARAR_SMART",
                        Commands.parallel(
                                // 1. Prendemos el shooter
                                s_ShooterFlywheels.runShooterCommand(3000),

                                // 2. Esperamos a que esté listo y disparamos
                                Commands.sequence(
                                        Commands.waitUntil(s_ShooterFlywheels::estaEnVelocidad),
                                        Commands.parallel(
                                                s_Indexer.encender(),
                                                s_IntakeRollers.movimiento() // (Aquí puedes usar tu nuevo método anti-atascos luego)
                                        )
                                )
                        ).withTimeout(2.5) // Límite de vida súper importante en Autónomo
                );
                
                NamedCommands.registerCommand("BAJAR_INTAKE", s_IntakeLift.bajarProtegido());
                NamedCommands.registerCommand("PRENDER_INDEXER", s_Indexer.encender());
                NamedCommands.registerCommand("MENEITO", s_Swerve.sacudirChasis());

                // -------------------------------------------------------------------
                // PESTAÑA DE DIAGNÓSTICO (SYSID & GYRO)
                // -------------------------------------------------------------------
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

                // -------------------------------------------------------------------
                // CONFIGURACIONES POR DEFECTO Y CHOOSER
                // -------------------------------------------------------------------
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Selector", autoChooser);

                s_Swerve.setDefaultCommand(new TeleopSwerve(
                                s_Swerve,
                                () -> -driver1.getLeftY(),
                                () -> -driver1.getLeftX(),
                                () -> driver1.getRightX(),
                                () -> driver1.getLeftTriggerAxis(),
                                () -> driver1.getHID().getLeftBumperButton(),
                                () -> driver1.getHID().getRightBumperButton(),
                                () -> driver1.getHID().getPOV()));

                if (RobotBase.isReal()) {
                        SmartDashboard.putData("Calibracion/Quasistatic Forward", new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward), Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Quasistatic Reverse", new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse), Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Dynamic Forward", new DeferredCommand(() -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward), Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Dynamic Reverse", new DeferredCommand(() -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse), Set.of(s_Swerve)));
                }

                s_ShooterAzimuth.setDefaultCommand(s_ShooterAzimuth.controlManualAzimuth(
                                () -> -driver2.getLeftX(),
                                () -> -driver2.getLeftY()));

                configureButtonBindings();
        }

        public Command alinearChasisEvasivo(DoubleSupplier translationX, DoubleSupplier translationY) {
                return Commands.run(() -> {
                        Pose2d robotPose = s_Swerve.getPose();
                        Rotation2d robotYaw = s_Swerve.getYaw();

                        Rotation2d anguloHaciaMeta = Rotation2d.fromRadians(
                                        Math.atan2(
                                                        Constants.shooter.kPosicionMeta.getY() - robotPose.getY(),
                                                        Constants.shooter.kPosicionMeta.getX() - robotPose.getX()));

                        // ESTRATEGIA DE 60 GRADOS:
                        // Le sumamos 30 grados al chasis para que mire "a la izquierda" del núcleo.
                        // Esto forzará a la torreta a irse a sus 60 grados (hacia la derecha) para compensar,
                        // dejándole 60 grados libres para cada lado en caso de que el piloto derrape.
                        Rotation2d anguloChasisDeseado = anguloHaciaMeta.plus(Rotation2d.fromDegrees(30.0));

                        double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.OIConstants.kStickDeadband);
                        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.OIConstants.kStickDeadband);

                        double rotacionPID = s_Swerve.headingController.calculate(robotYaw.getDegrees(), anguloChasisDeseado.getDegrees());
                        double rotacionLimitada = MathUtil.clamp(rotacionPID, -1.0, 1.0);

                        s_Swerve.drive(
                                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.kMaxSpeed),
                                        rotacionLimitada * Constants.Swerve.kMaxAngularVelocity,
                                        true, false);

                        SmartDashboard.putNumber("AutoAim/AnguloChasis_Deseado", anguloChasisDeseado.getDegrees());
                }, s_Swerve);
        }

        public Command vibrarDriver(CommandXboxController driverM, XboxController.RumbleType tipo, double magnitud, double tiempo) {
                return Commands.runEnd(
                                () -> driverM.getHID().setRumble(tipo, magnitud),
                                () -> driverM.getHID().setRumble(tipo, 0)).withTimeout(tiempo);
        }

        private void configureButtonBindings() {
                driver1.rightStick().onTrue(new InstantCommand(s_Swerve::zeroGyro));
                driver1.start().onTrue(
                                Commands.runOnce(() -> {
                                        s_Swerve.resetOdometry(new Pose2d(3.571, 4.0, Rotation2d.fromDegrees(180)));
                                }));
                driver1.leftStick().whileTrue(s_Swerve.sacudirChasis());
                
                // ==========================================================
                // DRIVER 1: CHASIS
                // ==========================================================
                // Al dejar presionado Left Trigger, el chasis se orienta automáticamente
                // mientras permite al piloto seguir derrapando/esquivando con los joysticks
                driver1.leftTrigger().whileTrue(
                                alinearChasisEvasivo(
                                                () -> -driver1.getLeftY(),
                                                () -> -driver1.getLeftX()));

                // ==========================================================
                // DRIVER 2: MECANISMOS (Torreta, Disparo y Fuego)
                // ==========================================================
                driver2.x().whileTrue(s_IntakeRollers.tragarPelotas());
                driver2.leftBumper().whileTrue(s_Indexer.encender());
                driver2.leftTrigger().whileTrue(s_Indexer.alRevez());
                driver2.a().onTrue(s_IntakeLift.bajarProtegido());
                driver2.b().onTrue(s_IntakeLift.subirProtegido());
                driver2.rightStick().whileTrue(s_ShooterFlywheels.testShooterDesdeDashboard());

                driver2.y().whileTrue(
                                Commands.parallel(s_ShooterFlywheels.testShooterDesdeDashboard(), Commands.sequence(
                                                Commands.waitUntil(
                                                                s_ShooterFlywheels::estaEnVelocidad),
                                                s_Indexer.dispararConAntiAtasco(s_ShooterFlywheels::detectoBajonPelota))));

                driver2.rightBumper().whileTrue(s_IntakeRollers.movimiento());
                driver2.povRight().onTrue(s_ShooterAzimuth.homingCero());
                
                driver2.povUp().toggleOnTrue(s_ShooterAzimuth.setAzimuthAngleCommand());
                
                // Al presionar Right Trigger, la torreta sigue al objetivo y las llantas se
                // prenden a 2800 RPM. Cuando lleguen a la velocidad, el indexer dispara.
                driver2.rightTrigger().whileTrue(
                                Commands.parallel(
                                                // ACCIÓN A: La torreta persiguiendo el núcleo a tiempo real
                                                s_ShooterAzimuth.autoApuntar(() -> {
                                                        Pose2d robotPose = s_Swerve.getPose();
                                                        Rotation2d robotYaw = s_Swerve.getYaw();

                                                        Rotation2d anguloHaciaMeta = Rotation2d.fromRadians(
                                                                        Math.atan2(
                                                                                        Constants.shooter.kPosicionMeta.getY() - robotPose.getY(),
                                                                                        Constants.shooter.kPosicionMeta.getX() - robotPose.getX()));

                                                        // Calculamos hacia dónde está la meta respecto a la trompa del robot
                                                        double anguloRelativo = anguloHaciaMeta.minus(robotYaw).getDegrees();

                                                        // SEGÚN TU DIBUJO: Le sumamos 90 porque el 90 de tu torreta es el Frente del robot
                                                        return anguloRelativo + 85.0;
                                                }),

                                                // ACCIÓN B: Preparar llantas y accionar Indexer
                                                s_ShooterFlywheels.runShooterCommand(2800) // Cambia esto al RPM fijo que quieras
                                                                .alongWith(
                                                                                Commands.sequence(
                                                                                                // Esperar a que las llantas de policarbonato lleguen a 2800
                                                                                                Commands.waitUntil(s_ShooterFlywheels::estaEnVelocidad),
                                                                                                
                                                                                                // FUEGO!
                                                                                                s_Indexer.dispararConAntiAtasco(s_ShooterFlywheels::estaEnVelocidad)
                                                                                )
                                                                )
                                )
                );
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
