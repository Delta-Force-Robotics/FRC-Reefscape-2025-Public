package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/** Deep cage climber: a position-controlled climber arm and a roller that pulls in the cage. */
public class ClimbSubsystem extends SubsystemBase {
  private final SparkFlex climberMotor = new SparkFlex(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
  private final SparkMax cageMotor = new SparkMax(ClimberConstants.kCageMotorID, MotorType.kBrushless);
  private final SparkClosedLoopController climberController = climberMotor.getClosedLoopController();

  public ClimbSubsystem() {
    configureMotors();
  }

  public Command moveClimber(double position) {
    return runOnce(() -> climberController.setReference(position, ControlType.kPosition));
  }

  /** Open-loop climber control, used as a backup if the position setpoints are off. */
  public Command runClimber(double power) {
    return runOnce(() -> climberMotor.set(power));
  }

  public Command runCageMotor(double power) {
    return runOnce(() -> cageMotor.set(power));
  }

  public double getClimberPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Position", getClimberPosition());
  }

  private void configureMotors() {
    var climberConfig = new SparkFlexConfig();
    climberConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
    climberConfig.closedLoop.pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var cageConfig = new SparkMaxConfig();
    cageConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    cageMotor.configure(cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
