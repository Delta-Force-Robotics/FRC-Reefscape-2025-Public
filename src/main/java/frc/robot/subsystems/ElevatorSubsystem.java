package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX frontElevatorMotor = new TalonFX(ElevatorConstants.kFrontMotorID, Constants.kCanBus);
    private final TalonFX backElevatorMotor = new TalonFX(ElevatorConstants.kBackMotorID, Constants.kCanBus);

    private final MotionMagicDutyCycle motionMagicDutyCycle = 
    new MotionMagicDutyCycle(0).withEnableFOC(true).withSlot(0);

    public ScoringTarget scoringTarget = ScoringTarget.L4;
    public AlgaePickUp algaePickUp = AlgaePickUp.FLOOR;
    public int scoringIncrement = 5;
    public int algaeIncrement = 1;
    public double multiplier = 1;

    public ElevatorSubsystem() {
        configureElevatorMotors();
        frontElevatorMotor.setControl(motionMagicDutyCycle.withPosition(ElevatorConstants.kInterPos));
    }

    public Command moveElevator(double position) {
        return this.runOnce(
            () -> frontElevatorMotor.setControl(motionMagicDutyCycle.withPosition(position)));
    }

    public Command goToTarget() {
        return this.runOnce(
            () -> frontElevatorMotor.setControl(motionMagicDutyCycle.withPosition(scoringTarget.getTargetRotations())));
    }

    public Command pickUpAlgae() {
        return this.runOnce(
            () -> frontElevatorMotor.setControl(motionMagicDutyCycle.withPosition(algaePickUp.getTargetRotations())));
    }

    public ScoringTarget getScoringTarget() {
        return this.scoringTarget;
    }

    public AlgaePickUp getAlgaePickUp() {
        return this.algaePickUp;
    }

    public double getElevatorPosition() {
        return frontElevatorMotor.getPosition().getValueAsDouble();
    }

    public double multiplier() {
        return multiplier;
    }

    public Command resetEncoder() {
        return this.runOnce(() -> frontElevatorMotor.setPosition(0));
    }

    public double setMultiplier(double multiplier) {
        this.multiplier = multiplier;
        return multiplier;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Elevator Position: ", frontElevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Scoring Target: ", getScoringTarget().toString());
        SmartDashboard.putString("Algae Pick Up", getAlgaePickUp().toString());
        SmartDashboard.putNumber("Elevator Rotations: ", scoringTarget.getTargetRotations());
        SmartDashboard.updateValues();
    }

    public enum ScoringTarget {
        L4(ElevatorConstants.kL4Pos),
        L3(ElevatorConstants.kL3Pos),
        L2(ElevatorConstants.kL2Pos),
        L1(ElevatorConstants.kL1Pos),
        BARGE(ElevatorConstants.kBargePos),
        PROCESSOR(ElevatorConstants.kProcessorPos);

        private final double targetRotations;

        ScoringTarget(double targetRotations) {
            this.targetRotations = targetRotations;
        }

        public double getTargetRotations() {
            return targetRotations;
        }
    }

    public enum AlgaePickUp {
        ALGAEL2(ElevatorConstants.kAlgaeL2Pos),
        ALGAEL3(ElevatorConstants.kAlgaeL3Pos),
        FLOOR(ElevatorConstants.kAlgaeFloorPos);

        private final double targetRotations;

        AlgaePickUp(double targetRotations) {
            this.targetRotations = targetRotations;
        }

        public double getTargetRotations() {
            return targetRotations;
        }
    }
    
    public void increaseScoringIncrement() {
        if (scoringIncrement < 6) {scoringIncrement++;}
        if (scoringIncrement == 1) {scoringTarget = ScoringTarget.PROCESSOR;}
        else if (scoringIncrement == 2) {scoringTarget = ScoringTarget.L1;}
        else if (scoringIncrement == 3) {scoringTarget = ScoringTarget.L2;}
        else if (scoringIncrement == 4) {scoringTarget = ScoringTarget.L3;}
        else if (scoringIncrement == 5) {scoringTarget = ScoringTarget.L4;}
        else if (scoringIncrement == 6) {scoringTarget = ScoringTarget.BARGE;}
    }

    public void decreaseScoringIncrement() {
        if (scoringIncrement > 1) {scoringIncrement--;}
        if (scoringIncrement == 1) {scoringTarget = ScoringTarget.PROCESSOR;}
        else if (scoringIncrement == 2) {scoringTarget = ScoringTarget.L1;}
        else if (scoringIncrement == 3) {scoringTarget = ScoringTarget.L2;}
        else if (scoringIncrement == 4) {scoringTarget = ScoringTarget.L3;}
        else if (scoringIncrement == 5) {scoringTarget = ScoringTarget.L4;}
        else if (scoringIncrement == 6) {scoringTarget = ScoringTarget.BARGE;}
    }

    public void increaseAlgaeIncrement() {
        if (algaeIncrement < 3) {algaeIncrement++;}
        if (algaeIncrement == 1) {algaePickUp = AlgaePickUp.FLOOR;}
        else if (algaeIncrement == 2) {algaePickUp = AlgaePickUp.ALGAEL2;}
        else if (algaeIncrement == 3) {algaePickUp = AlgaePickUp.ALGAEL3;}
    }

    public void decreaseAlgaeIncrement() {
        if (algaeIncrement > 1) {algaeIncrement--;}
        if (algaeIncrement == 1) {algaePickUp = AlgaePickUp.FLOOR;}
        else if (algaeIncrement == 2) {algaePickUp = AlgaePickUp.ALGAEL2;}
        else if (algaeIncrement == 3) {algaePickUp = AlgaePickUp.ALGAEL3;}
    }

    public void configureElevatorMotors() {
        // Initialize TalonFX configs
        var talonFXConfigs = new TalonFXConfiguration();

        // Set the controller gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kI = ElevatorConstants.kI;
        slot0Configs.kD = ElevatorConstants.kD;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.kAcceleration;
        
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        frontElevatorMotor.getConfigurator().apply(talonFXConfigs);
        backElevatorMotor.setControl(new Follower(frontElevatorMotor.getDeviceID(), true));

        // Reset motor encoders position
        frontElevatorMotor.setPosition(0);
        backElevatorMotor.setPosition(0);
    }
}