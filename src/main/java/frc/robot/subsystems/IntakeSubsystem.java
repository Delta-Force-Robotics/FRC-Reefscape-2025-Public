package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
    private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    private SparkFlexConfig pivotMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

    public IntakeSubsystem() {
        configurePivotMotor();
        configureIntakeMotor();
    }

    public Command runIntake(double power) {
        return this.runOnce(() -> intakeMotor.set(power));
    }

    public Command moveIntake(double position) {
        return this.runOnce(() -> pivotController.setReference(position, ControlType.kPosition));
    }

    public double getIntakePosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public Command resetEncoder() {
        return this.runOnce(() -> pivotMotor.getEncoder().setPosition(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Position: ", pivotMotor.getEncoder().getPosition());
        SmartDashboard.updateValues();
    }

    public void configurePivotMotor() {
        pivotMotorConfig
         .inverted(true)
         .smartCurrentLimit(IntakeConstants.kSmartCurrentLimitPivot)
         .idleMode(IdleMode.kBrake);
         pivotMotorConfig.closedLoop
         .p(IntakeConstants.kP)
         .i(IntakeConstants.kI)
         .d(IntakeConstants.kD);
         
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configureIntakeMotor() {
        intakeMotorConfig
         .inverted(true)
         .smartCurrentLimit(IntakeConstants.kSmartCurrentLimitIntake)
         .idleMode(IdleMode.kCoast);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}