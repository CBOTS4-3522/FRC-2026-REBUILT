package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.swerve.SwerveBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotBase; // Para la condición if (RobotBase.isReal())

public class RobotContainer {
        /* Shuffleboard */
        public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        /* Controllers */
        private final CommandXboxController driver1 = new CommandXboxController(Constants.OIConstants.kDriver1Port);
        private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

        /* Subsystems */
        private final SwerveBase s_Swerve;
        private final Shooter s_Shooter;

        /* Autos */

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                s_Swerve = new SwerveBase();

                if (Robot.isReal()) {
                        // Si es el robot real, usa los SparkMax
                        s_Shooter = new Shooter(new ShooterIOSparkMax());
                } else {
                        // Si es simulación, podrías usar una clase ShooterIOSim (que haríamos después)
                        // O un objeto vacío para que no truene:
                        s_Shooter = new Shooter(new ShooterIO() {
                        }) {
                        };
                }

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
                                new DeferredCommand(() -> s_Shooter.sysIdQuasistatic(Direction.kForward),
                                                Set.of(s_Shooter)))
                                .withSize(2, 1)
                                .withPosition(0, 2); // Fila 2, Columna 0

                // Quasistatic Reverse (Rampa de voltaje suave negativa)
                diagTab.add("Shooter QS Rev",
                                new DeferredCommand(() -> s_Shooter.sysIdQuasistatic(Direction.kReverse),
                                                Set.of(s_Shooter)))
                                .withSize(2, 1)
                                .withPosition(2, 2); // Fila 2, Columna 2

                // Dynamic Forward (Salto de voltaje positivo - Step)
                diagTab.add("Shooter Dyn Fwd",
                                new DeferredCommand(() -> s_Shooter.sysIdDynamic(Direction.kForward),
                                                Set.of(s_Shooter)))
                                .withSize(2, 1)
                                .withPosition(0, 3); // Fila 3, Columna 0

                // Dynamic Reverse (Salto de voltaje negativo - Step)
                diagTab.add("Shooter Dyn Rev",
                                new DeferredCommand(() -> s_Shooter.sysIdDynamic(Direction.kReverse),
                                                Set.of(s_Shooter)))
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
                                                () -> driver1.getHID().getLeftBumperButton()));

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

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}