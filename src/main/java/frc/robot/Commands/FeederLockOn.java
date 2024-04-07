package frc.robot.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class FeederLockOn extends Command {
  /** Creates a new NoteLockOn. */
  private final DriveSubsystem m_DriveSubsystem;
  private final DoubleSupplier m_XSpeedSupplier;
  private final DoubleSupplier m_YSpeedSupplier;
  private final PIDController m_ThetaController;
  private final double targetAngle;
  public FeederLockOn(DriveSubsystem m_DriveSubsystem, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_XSpeedSupplier = xSpeedSupplier;
    this.m_YSpeedSupplier = ySpeedSupplier;
    addRequirements(m_DriveSubsystem);
    if(DriverStation.getAlliance().isPresent()){
    targetAngle = DriverStation.getAlliance().get() ==  DriverStation.Alliance.Red ? 
      315 :
      225;
    } else {
      targetAngle = 315;
    }


    m_ThetaController = new PIDController(2, 0, 0.001);
    m_ThetaController.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //The set point is 0 since that is where the heading of the note is located at
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ThetaController.setSetpoint(targetAngle);
    double thetaOutput = m_ThetaController.calculate(m_DriveSubsystem.getPose().getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)).getRadians());
    m_DriveSubsystem.drive(
        m_XSpeedSupplier.getAsDouble(),
        m_YSpeedSupplier.getAsDouble(), 
        thetaOutput,
        true,
        false);

        

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_DriveSubsystem.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
