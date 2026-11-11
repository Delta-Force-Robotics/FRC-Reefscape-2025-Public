package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkFlex climberMotor = new SparkFlex(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    private final SparkMax cageMotor = new SparkMax(ClimberConstants.kCageMotorID, MotorType.kBrushless);
    
    private SparkFlexConfig climberConfig = new SparkFlexConfig();
    private SparkMaxConfig cageConfig = new SparkMaxConfig();

    private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();

    public ClimbSubsystem() {
        configureClimber();
        configureCage();
    }

    public Command moveClimber(double position) {
        return this.runOnce(() -> climberController.setReference(position, ControlType.kPosition));
    }

    public Command manualClimb(double power) {
        return this.runOnce(() -> climberMotor.set(power));
    }

    public Command runCageMotor(double power) {
        return this.runOnce(() -> cageMotor.set(power));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position: ", climberMotor.getEncoder().getPosition());
        SmartDashboard.updateValues();
    }

    public void configureClimber() {
        climberConfig
         .inverted(true)
         .idleMode(IdleMode.kBrake)
         .smartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
         climberConfig.closedLoop
         .pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

         climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configureCage() {
        cageConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        cageMotor.configure(cageConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}