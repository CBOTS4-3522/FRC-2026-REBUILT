package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveBase;
import frc.robot.commands.VisionAlignCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.RobotBase; // Para la condición if (RobotBase.isReal())

public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    /* Controllers */
    private final XboxController driver1 = new XboxController(Constants.OIConstants.kDriver1Port);

    /* Subsystems */
    private final SwerveBase s_Swerve;

    private final VisionAlignCommand visionAlignCommand;

    ///// Driver 1////////////
    private final int translationX = XboxController.Axis.kLeftY.value;
    private final int translationY = XboxController.Axis.kLeftX.value;
    private final int rotation = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driver1, XboxController.Button.kRightStick.value);
    private final DoubleSupplier turbo = () -> driver1.getRawAxis(2);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        s_Swerve = new SwerveBase();

        ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");

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

        /*
         * diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro)
         * .withSize(2, 2).withPosition(4, 0);
         * 
         * // 3. Agregamos el Gyro y el Selector de Autos para tener todo a la mano
         * diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro)
         * .withSize(2, 2).withPosition(4, 0);
         */

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
                        () -> -driver1.getRawAxis(translationX),
                        () -> -driver1.getRawAxis(translationY),
                        () -> driver1.getRawAxis(rotation),
                        turbo,
                        () -> driver1.getRawButtonPressed(XboxController.Button.kLeftBumper.value)));

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
        visionAlignCommand = new VisionAlignCommand(s_Swerve, driver1);
        
        
        // Configure the button bindings
        configureButtonBindings();
        
    }

    private void configureButtonBindings() {

        // Reset Gyro
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
