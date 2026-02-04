package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.SwerveBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotBase; // Para la condición if (RobotBase.isReal())

public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    /* Controllers */
    private final Joystick driver1 = new Joystick(Constants.OIConstants.kDriver1Port);
    private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

    /* Subsystems */
    private final SwerveBase s_Swerve;
    private final Intake s_Intake;
    private final Shooter s_Shooter;

    ///// Driver 1////////////
    private final int translationX = XboxController.Axis.kLeftY.value;
    private final int translationY = XboxController.Axis.kLeftX.value;
    private final int rotation = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kRightStick.value);
    private final JoystickButton resetPose = new JoystickButton(driver1, XboxController.Button.kStart.value);
    private final JoystickButton xStance = new JoystickButton(driver1, XboxController.Button.kX.value);
    private final DoubleSupplier turbo = () -> driver1.getRawAxis(2);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        s_Swerve = new SwerveBase();
        s_Intake = new Intake();
        s_Shooter = new Shooter();

        NamedCommands.registerCommand("TRAGAR", s_Intake.intakeON());
        NamedCommands.registerCommand("LLENO", s_Intake.intakeOFF());
        NamedCommands.registerCommand("SUBIR INTAKE", s_Intake.upAuto());
        NamedCommands.registerCommand("BAJAR INTAKE", s_Intake.downAuto());

        SmartDashboard.putNumber("Shooter/VelocidadTest", 0.0);

        ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");
        SmartDashboard.putNumber("Intake/VelocidadI", 1.0);

        SmartDashboard.putNumber("Intake/UP", 1.0);
        SmartDashboard.putNumber("Intake/DOWN", -1.0);

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

<<<<<<< HEAD
        diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro)
                .withSize(2, 2).withPosition(4, 0);
=======
    SmartDashboard.putData("Auto mode", autoChooser);
    
>>>>>>> simulation/path-planner

        // Autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        /* Swerve */
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver1.getRawAxis(translationX),
                        () -> -driver1.getRawAxis(translationY),
                        () -> driver1.getRawAxis(rotation),
                        turbo,
                        () -> driver1.getRawButtonPressed(XboxController.Button.kLeftBumper.value),
                        () -> driver1.getRawButtonPressed(XboxController.Button.kRightBumper.value)));

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

    private void configureButtonBindings() {

        // Reset Gyro
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetPose.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new edu.wpi.first.math.geometry.Pose2d())));
        xStance.whileTrue(new RunCommand(() -> s_Swerve.wheelsIn(), s_Swerve));
        driver2.y().whileTrue(s_Shooter.RunShooter());
        driver2.x().toggleOnTrue(s_Intake.runIntake());
        driver2.b().whileTrue(s_Intake.up());
        driver2.a().whileTrue(s_Intake.down());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
