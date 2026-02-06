package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
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
    private final Intake s_Intake;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        s_Swerve = new SwerveBase();
        s_Intake = new Intake();

        ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");
        SmartDashboard.putNumber("Intake/VelocidadTest", 1.0);

        // USAR DEFERREDCOMMAND AQUÍ TAMBIÉN
        diagTab.add("Quasistatic Forward",
                new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kForward), Set.of(s_Swerve)))
                .withSize(2, 1).withPosition(0, 0);

        diagTab.add("Quasistatic Reverse",
                new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(Direction.kReverse), Set.of(s_Swerve)))
                .withSize(2, 1).withPosition(2, 0);

        diagTab.add("Dynamic Forward",
                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kForward), Set.of(s_Swerve)))
                .withSize(2, 1).withPosition(0, 1);

        diagTab.add("Dynamic Reverse",
                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kReverse), Set.of(s_Swerve)))
                .withSize(2, 1).withPosition(2, 1);

        diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro)
                .withSize(2, 2).withPosition(4, 0);

        // Autos
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Giro", autoChooser);
        SmartDashboard.putData("Frente", autoChooser);
        SmartDashboard.putData("Derecha", autoChooser);
        SmartDashboard.putData("Prueba", autoChooser);

        /* Swerve */
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver1.getLeftY(), // Traslación X (Adelante/Atrás)
                        () -> -driver1.getLeftX(), // Traslación Y (Izquierda/Derecha)
                        () -> -driver1.getRightX(), // Rotación
                        () -> driver1.getLeftTriggerAxis(), // Turbo (Gatillo Izquierdo)
                        () -> driver1.getHID().getLeftBumper() // Robot Centric (Botón LB)
                ));

        // En RobotContainer.java, dentro del constructor public RobotContainer()

        // Dentro del constructor de RobotContainer
        if (RobotBase.isReal()) {
            SmartDashboard.putData("Calibracion/Quasistatic Forward",
                    new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                            Set.of(s_Swerve)));

            SmartDashboard.putData("Calibracion/Quasistatic Reverse",
                    new DeferredCommand(() -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                            Set.of(s_Swerve)));

            SmartDashboard.putData("Calibracion/Dynamic Forward",
                    new DeferredCommand(() -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward),
                            Set.of(s_Swerve)));

            SmartDashboard.putData("Calibracion/Dynamic Reverse",
                    new DeferredCommand(() -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                            Set.of(s_Swerve)));
        }
        // Configure the button bindings
        configureButtonBindings();
    }

    public Command vibrarDriver1() {
        return Commands.runEnd(
                () -> {
                    // Driver 1 (Ahora es fácil acceder al HID)
                    driver1.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0);
                    // Driver 2
                },
                () -> {
                    driver1.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5);
    }

    public Command vibrarDriver2() {
        return Commands.runEnd(
                () -> {
                    driver2.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0);
                },
                () -> {
                    driver2.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5);
    }

    private void configureButtonBindings() {

        // Reset Gyro
        driver1.rightStick().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        s_Intake.getTriggerPelota().onTrue(Commands.parallel(
            vibrarDriver1(),
            vibrarDriver2()
            ));

        // Intake
        driver2.x().toggleOnTrue(s_Intake.tragarPelotas());
        driver2.y().whileTrue(s_Intake.escupirPelotas());
        driver2.b().onTrue(s_Intake.subir());
        driver2.a().onTrue(s_Intake.bajar());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
