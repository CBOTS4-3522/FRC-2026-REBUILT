package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class TeleopSwerve extends Command {
    private final SwerveBase s_Swerve;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;
    private final DoubleSupplier rotation;
    private final DoubleSupplier turbo;
    private final BooleanSupplier toggleRobotCentric;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Swerve.kSlewRateLimit);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Swerve.kSlewRateLimit);

    private boolean robotCentric = false;
    private boolean lastButtonState = false;

    public TeleopSwerve(
            SwerveBase s_Swerve,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            DoubleSupplier turbo,
            BooleanSupplier toggleRobotCentric) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.turbo = turbo;
        this.toggleRobotCentric = toggleRobotCentric;
    }

    @Override
    public void execute() {

        // 1. Leer valores de joystick con deadband
        double xRaw = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.OIConstants.kStickDeadband);
        double yRaw = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.OIConstants.kStickDeadband);
        double rotationVal = MathUtil.applyDeadband(-rotation.getAsDouble(), Constants.OIConstants.kStickDeadband);

        // 2. APLICAR EL FILTRO (Capacitor Digital)
        double xFiltered = xLimiter.calculate(xRaw);
        double yFiltered = yLimiter.calculate(yRaw);

    // Turbo para ajustar velocidad (manteniendo tu lógica actual)
        boolean speedCutoffVal = turbo.getAsDouble() <= 0.1;
        
        // Leer valores de joystick con deadband
        double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.OIConstants.kStickDeadband);
        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.OIConstants.kStickDeadband);
        

       

        // Alternar entre orientado al campo y orientado al robot
        boolean currentButtonState = toggleRobotCentric.getAsBoolean();
        if (currentButtonState && !lastButtonState) {
            robotCentric = !robotCentric;
        }
        lastButtonState = currentButtonState;

        // Actualizar en el SmartDashboard
        SmartDashboard.putNumber("translationVal", translationVal);
        SmartDashboard.putNumber("strafeVal", strafeVal);
        SmartDashboard.putNumber("rotationVal", rotationVal);
        SmartDashboard.putBoolean("Robot Centric Mode", robotCentric);

        s_Swerve.drive(
            new Translation2d(xFiltered, yFiltered)
                    .times(Constants.Swerve.kMaxSpeed)
                    .times(speedCutoffVal ? 1 : 0.5),
            rotationVal * Constants.Swerve.kMaxAngularVelocity * (speedCutoffVal ? 0.5 : 1),
            !robotCentric, Constants.Swerve.kIsOpenLoopTeleopSwerve
        );
    }
}
