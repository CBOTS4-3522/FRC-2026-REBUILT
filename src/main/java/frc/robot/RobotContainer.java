package frc.robot;

// 1. Librerías base de Java
import java.util.Set;

// 2. Librerías de Terceros (PathPlanner)
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 5. WPILib: Framework de Comandos
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// 6. Archivos Locales (Subsistemas y Comandos de la 3522)
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.intake.IntakeLift;
import frc.robot.subsystems.intake.IntakeLiftIO;
import frc.robot.subsystems.intake.IntakeLiftOIReal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollersIO;
import frc.robot.subsystems.intake.IntakeRollersIOReal;
import frc.robot.subsystems.shooter.ShooterAzimuth;
import frc.robot.subsystems.shooter.ShooterAzimuthIOReal;
import frc.robot.subsystems.shooter.ShooterAzimuthIO;
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterFlywheelsIO;
import frc.robot.subsystems.shooter.ShooterFlywheelsIOReal;
import frc.robot.subsystems.swerve.SwerveBase;

public class RobotContainer {
        /* Shuffleboard */
        public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        /* Controllers */
        private final CommandXboxController driver1 = new CommandXboxController(Constants.OIConstants.kDriver1Port);
        private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

        /* Subsystems */
        private final SwerveBase s_Swerve;
        private final ShooterAzimuth s_ShooterAzimuth;
        private final ShooterFlywheels s_ShooterFlywheels;
        private final Indexer s_Indexer;
        private final IntakeLift s_IntakeLift;
        private final IntakeRollers s_IntakeRollers;

        /* Autos */

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                s_Swerve = new SwerveBase();
                s_Indexer = new Indexer();

                // ShooterIO shooterIO; // 1. Declaramos la interfaz temporal

                if (Robot.isReal()) {
                        // Si es el robot real, usa los SparkMax
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIOReal());
                } else {
                        // Si es simulación, podrías usar una clase ShooterIOSim (que haríamos después)
                        // O un objeto vacío para que no truene:
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIO() {
                        }) {
                        };
                }

                if (Robot.isReal()) {
                        // Si es el robot real, usa los SparkMax
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOReal());
                } else {
                        // Si es simulación, podrías usar una clase ShooterIOSim (que haríamos después)
                        // O un objeto vacío para que no truene:
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIO() {
                        }) {
                        };
                }

                IntakeLiftIO intakeLiftIO; // 1. Declaramos la interfaz temporal

                if (RobotBase.isReal()) {
                        // 2. Si es el robot de verdad, creamos la IO Real
                        intakeLiftIO = new IntakeLiftOIReal();
                } else {
                        // 3. Si es simulador, usamos la IO Sim (o vacía por ahora si no la tienes)
                        // Por ahora puedes poner: intakeIO = new IntakeIO() {};
                        // O mejor aún, crea el archivo IntakeIOSim.java después.
                        intakeLiftIO = new IntakeLiftIO() {
                                @Override
                                public void setVoltajeLift(double volts) {
                                }

                                @Override
                                public void stopLift() {
                                }
                        }; // IO "muda" para que no truene el sim
                }
                s_IntakeLift = new IntakeLift(intakeLiftIO);

                IntakeRollersIO intakeRollersIO; // 1. Declaramos la interfaz temporal

                if (RobotBase.isReal()) {
                        // 2. Si es el robot de verdad, creamos la IO Real
                        intakeRollersIO = new IntakeRollersIOReal();
                } else {
                        // 3. Si es simulador, usamos la IO Sim (o vacía por ahora si no la tienes)
                        // Por ahora puedes poner: intakeIO = new IntakeIO() {};
                        // O mejor aún, crea el archivo IntakeIOSim.java después.
                        intakeRollersIO = new IntakeRollersIO() {
                                @Override
                                public void setVoltajeRodillos(double volts) {
                                }

                                @Override
                                public void stopRodillos() {
                                }

                        }; // IO "muda" para que no truene el sim
                }
                s_IntakeRollers = new IntakeRollers(intakeRollersIO);

                // System ID
                ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");

                // USAR DEFERREDCOMMAND

                diagTab.add("Quasistatic Forward",
                                new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kForward),
                                                Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(0, 0);

                diagTab.add("Quasistatic Reverse",
                                new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kReverse),
                                                Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(2, 0);

                diagTab.add("Dynamic Forward",
                                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kForward), Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(0, 1);

                diagTab.add("Dynamic Reverse",
                                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kReverse), Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(2, 1);

                diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro)
                                .withSize(2, 2).withPosition(4, 0);

                // ... (Código existente del Swerve SysId) ...

                // ==========================================================
                // BOTONES SYSID SHOOTER (FLYWHEEL)
                // ==========================================================

                // Quasistatic Forward (Rampa de voltaje suave positiva)
                diagTab.add("Shooter QS Fwd",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kForward),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1)
                                .withPosition(0, 2); // Fila 2, Columna 0

                // Quasistatic Reverse (Rampa de voltaje suave negativa)
                diagTab.add("Shooter QS Rev",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kReverse),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1)
                                .withPosition(2, 2); // Fila 2, Columna 2

                // Dynamic Forward (Salto de voltaje positivo - Step)
                diagTab.add("Shooter Dyn Fwd",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdDynamic(Direction.kForward),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1)
                                .withPosition(0, 3); // Fila 3, Columna 0

                // Dynamic Reverse (Salto de voltaje negativo - Step)
                diagTab.add("Shooter Dyn Rev",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdDynamic(Direction.kReverse),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1)
                                .withPosition(2, 3); // Fila 3, Columna 2

                // Autos

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Selector", autoChooser);

                /* Swerve */
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -driver1.getLeftY(), // Traslación X (Adelante/Atrás)
                                                () -> -driver1.getLeftX(), // Traslación Y (Izquierda/Derecha)
                                                () -> driver1.getRightX(), // Rotación
                                                () -> driver1.getLeftTriggerAxis(), // Turbo (Gatillo Izquierdo)
                                                () -> driver1.getHID().getLeftBumperButton(),
                                                () -> driver1.getHID().getRightBumperButton(),
                                                () -> driver1.getHID().getPOV()));

                // Elastic
                if (RobotBase.isReal()) {
                        SmartDashboard.putData("Calibracion/Quasistatic Forward",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdQuasistatic(
                                                                        SysIdRoutine.Direction.kForward),
                                                        Set.of(s_Swerve)));

                        SmartDashboard.putData("Calibracion/Quasistatic Reverse",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdQuasistatic(
                                                                        SysIdRoutine.Direction.kReverse),
                                                        Set.of(s_Swerve)));

                        SmartDashboard.putData("Calibracion/Dynamic Forward",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward),
                                                        Set.of(s_Swerve)));

                        SmartDashboard.putData("Calibracion/Dynamic Reverse",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                                                        Set.of(s_Swerve)));
                }
                s_ShooterAzimuth.setDefaultCommand(
                                s_ShooterAzimuth.controlManualAzimuth(
                                                () -> driver2.getLeftX(), // Izquierda/Derecha para la torreta
                                                () -> -driver2.getLeftY() // Arriba/Abajo para el chamfle (Negativo para
                                                                          // que Arriba sume grados)
                                ));
                // Configure the button bindings
                configureButtonBindings();
        }

        public Command vibrarDriver(CommandXboxController driverM, XboxController.RumbleType tipo, double magnitud,
                        double tiempo) {
                return Commands.runEnd(
                                () -> {

                                        driverM.getHID().setRumble(tipo, magnitud);

                                },
                                () -> {
                                        driverM.getHID().setRumble(tipo, 0);
                                }).withTimeout(tiempo);
        }

        private void configureButtonBindings() {

                // Reset Gyro
                driver1.rightStick().onTrue(new InstantCommand(s_Swerve::zeroGyro));
                driver1.start().onTrue(
                                Commands.runOnce(() -> {
                                        // Asumiendo que al chocar, el frente del robot mira hacia los 180 grados.
                                        // Cambia el 180 si el robot choca de reversa o de lado.
                                        s_Swerve.resetOdometry(new Pose2d(3.571, 4.0, Rotation2d.fromDegrees(180)));
                                }));
                driver1.leftStick().whileTrue(s_Swerve.sacudirChasis());

                driver2.x().whileTrue(s_IntakeRollers.tragarPelotas());
                // driver2.b().whileTrue(s_Shooter.runShooterCommand(4000));
                driver2.leftBumper().whileTrue(s_Indexer.encender());
                driver2.leftTrigger().whileTrue(s_Indexer.alRevez());
                driver2.a().onTrue(s_IntakeLift.bajarProtegido());
                driver2.b().onTrue(s_IntakeLift.subirProtegido());
                driver2.rightStick().whileTrue(s_ShooterFlywheels.testShooterDesdeDashboard());
                driver2.y().toggleOnTrue(
                                // 1. Prende el Shooter a 2000 RPM (Se queda corriendo)
                                s_ShooterFlywheels.testShooterDesdeDashboard()
                                                .alongWith(
                                                                // 2. EN PARALELO: Vigila y dispara
                                                                Commands.sequence(
                                                                                Commands.waitUntil(
                                                                                                s_ShooterFlywheels::estaEnVelocidad), // Espera
                                                                                s_Indexer.encender() // pacientemente...
                                                                // ¡Fuego!
                                                                )));
                driver2.rightBumper().whileTrue(s_IntakeRollers.movimiento());
                // driver2.b().toggleOnTrue(s_IntakeRollers.tragarPelotas());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}