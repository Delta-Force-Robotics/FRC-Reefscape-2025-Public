package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Ground coral intake: a position-controlled pivot and a roller. */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
  private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  public IntakeSubsystem() {
    configureMotors();
  }

  public Command runIntake(double power) {
    return runOnce(() -> rollerMotor.set(power));
  }

  public Command moveIntake(double position) {
    return runOnce(() -> pivotController.setReference(position, ControlType.kPosition));
  }

  public double getPivotPosition() {
    return pivotMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot Position", getPivotPosition());
  }

  private void configureMotors() {
    var pivotConfig = new SparkFlexConfig();
    pivotConfig
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.kSmartCurrentLimitPivot)
        .idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rollerConfig = new SparkFlexConfig();
    rollerConfig
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake)
        .idleMode(IdleMode.kCoast);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
