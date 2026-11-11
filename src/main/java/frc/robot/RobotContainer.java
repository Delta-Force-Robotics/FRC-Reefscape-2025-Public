// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate =
      RotationsPerSecond.of(DriveConstants.kMaxAngularRateRps).in(RadiansPerSecond);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final GripperSubsystem gripper = new GripperSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ClimbSubsystem climber = new ClimbSubsystem();

  private final Telemetry telemetry = new Telemetry(maxSpeed);

  private final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);
  /** Backup controller for running the climber open loop. */
  private final CommandPS5Controller climbOperator = new CommandPS5Controller(2);

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * DriveConstants.kDeadband)
          .withRotationalDeadband(maxAngularRate * DriveConstants.kDeadband)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SendableChooser<Command> autoChooser;

  /** Drive speed is reduced while the elevator is raised to keep the robot stable. */
  private double driveSpeedMultiplier = 1;

  private ScoringTarget scoringTarget = ScoringTarget.L4;
  private AlgaePickup algaePickup = AlgaePickup.FLOOR;

  public RobotContainer() {
    registerAutoEvents();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureDriveBindings();
    configureMechanismBindings();
    configureOperatorBindings();
    publishSelections();

    drivetrain.registerTelemetry(telemetry::telemeterize);

    // Warm up PathPlanner so the first auto path does not stall on JIT compilation.
    FollowPathCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Event markers referenced by the PathPlanner paths in deploy/pathplanner. */
  private void registerAutoEvents() {
    new EventTrigger("ExtendScoringL4")
        .onTrue(extendTo(ElevatorConstants.kL4Pos, ArmConstants.kL4Pos));

    new EventTrigger("ScoreCoralL4")
        .onTrue(
            Commands.sequence(
                arm.moveArm(ArmConstants.kAutoL4ReleasePos),
                Commands.waitSeconds(0.15),
                gripper.setIntakingAlgae(false),
                gripper.runGripper(GripperConstants.kOuttakePow)));

    new EventTrigger("RetractScore")
        .onTrue(
            Commands.sequence(
                arm.moveArm(ArmConstants.kHomePos),
                Commands.waitSeconds(0.2),
                elevator.moveElevator(ElevatorConstants.kWaitForCoralPos),
                gripper.runGripper(GripperConstants.kStopPow)));

    new EventTrigger("StartIntake").onTrue(startIntake());

    new EventTrigger("Reverse&StopIntake").onTrue(ejectAndStowIntake(0.2));

    new EventTrigger("IntakeAlgae2")
        .onTrue(extendTo(ElevatorConstants.kAlgaeL2Pos, ArmConstants.kAlgaeL2Pos));

    new EventTrigger("IntakeAlgae3")
        .onTrue(
            Commands.sequence(
                gripper.runGripper(GripperConstants.kIntakePow),
                extendTo(ElevatorConstants.kAlgaeL3Pos, ArmConstants.kAlgaeL3Pos)));

    new EventTrigger("ScoreAlgae")
        .onTrue(extendTo(ElevatorConstants.kBargePos, ArmConstants.kBargePos));

    new EventTrigger("ReleaseAlgae").onTrue(gripper.runGripper(GripperConstants.kOuttakePow));
  }

  private void configureDriveBindings() {
    // Idle while disabled so the configured neutral mode is applied to the drive motors.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * maxSpeed * driveSpeedMultiplier)
                    .withVelocityY(-driver.getLeftX() * maxSpeed * driveSpeedMultiplier)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate * driveSpeedMultiplier)));

    driver.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driver.L2().whileTrue(new AlignToReefTagRelative(false, drivetrain));
    driver.R2().whileTrue(new AlignToReefTagRelative(true, drivetrain));
  }

  private void configureMechanismBindings() {
    // Coral reached the end of the indexer: hand it off to the gripper and stow the intake.
    new Trigger(indexer::hasCoral)
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.1),
                gripper.runGripper(GripperConstants.kIntakePow),
                arm.moveArm(ArmConstants.kHomePos),
                elevator.moveElevator(ElevatorConstants.kHomePos),
                Commands.waitSeconds(0.15),
                gripper.runGripper(GripperConstants.kCoralPassivePow),
                elevator.moveElevator(ElevatorConstants.kInterPos),
                ejectAndStowIntake(0.1)));

    // Algae secured in the gripper: hold it with light power and tuck the arm in.
    new Trigger(gripper::hasAlgae)
        .onTrue(
            Commands.sequence(
                gripper.runGripper(GripperConstants.kAlgaePassivePow),
                setDriveSpeedMultiplier(1),
                arm.moveArm(ArmConstants.kAlgaeHoldPos)));

    // Deploy the intake and wait for coral.
    driver.cross()
        .onTrue(
            Commands.sequence(
                gripper.setIntakingAlgae(false),
                elevator.moveElevator(ElevatorConstants.kInterPos),
                intake.moveIntake(IntakeConstants.kParallelPos),
                Commands.waitSeconds(0.3),
                startIntake(),
                arm.moveArm(ArmConstants.kHomePos),
                elevator.moveElevator(ElevatorConstants.kWaitForCoralPos)));

    // Stow the intake, briefly reversing the rollers to clear anything stuck.
    driver.circle()
        .onTrue(
            Commands.sequence(
                intake.moveIntake(IntakeConstants.kStowedPos),
                intake.runIntake(IntakeConstants.kOuttakePow),
                indexer.runIndexer(IndexerConstants.kOuttakePow),
                Commands.waitSeconds(0.35),
                stopIntakeRollers()));

    // Pick up algae from the selected location.
    driver.square()
        .onTrue(
            Commands.sequence(
                gripper.setIntakingAlgae(true),
                elevator.moveElevator(ElevatorConstants.kInterPos),
                Commands.waitSeconds(0.25),
                arm.moveArm(() -> algaePickup.armPos),
                Commands.waitSeconds(0.15),
                elevator.moveElevator(() -> algaePickup.elevatorPos),
                gripper.runGripper(GripperConstants.kIntakePow)));

    // Raise to the selected scoring target.
    driver.L1()
        .onTrue(
            Commands.sequence(
                setDriveSpeedMultiplier(DriveConstants.kScoringSpeedMultiplier),
                elevator.moveElevator(ElevatorConstants.kInterPos),
                Commands.waitSeconds(0.25),
                arm.moveArm(() -> scoringTarget.armPos),
                Commands.waitSeconds(0.15),
                elevator.moveElevator(() -> scoringTarget.elevatorPos)));

    // Release the game piece, back away from the reef and retract.
    driver.R1()
        .onTrue(
            Commands.sequence(
                setDriveSpeedMultiplier(1),
                arm.moveArm(() -> scoringTarget.armReleasePos()),
                Commands.waitSeconds(0.15),
                gripper.setIntakingAlgae(false),
                gripper.runGripper(GripperConstants.kOuttakePow),
                Commands.waitSeconds(0.2),
                // Proxied so the drivetrain is only taken from the driver while backing off.
                backOffFromReef().asProxy(),
                Commands.waitSeconds(0.25),
                elevator.moveElevator(ElevatorConstants.kInterPos),
                Commands.waitSeconds(0.1),
                arm.moveArm(ArmConstants.kHomePos),
                Commands.waitSeconds(0.25),
                elevator.moveElevator(ElevatorConstants.kWaitForCoralPos),
                gripper.runGripper(GripperConstants.kStopPow)));

    driver.povLeft()
        .onTrue(
            Commands.sequence(
                climber.moveClimber(ClimberConstants.kDeployPos),
                climber.runCageMotor(ClimberConstants.kCageIntakePow)));

    driver.povRight()
        .onTrue(
            Commands.sequence(
                climber.moveClimber(ClimberConstants.kClimbPos),
                climber.runCageMotor(ClimberConstants.kCageStopPow)));
  }

  private void configureOperatorBindings() {
    // Emergency stop and stow for the intake.
    operator.circle()
        .onTrue(
            Commands.sequence(
                stopIntakeRollers(), intake.moveIntake(IntakeConstants.kStowedPos)));

    operator.povUp().onTrue(Commands.runOnce(() -> selectScoringTarget(scoringTarget.next())));
    operator.povDown().onTrue(Commands.runOnce(() -> selectScoringTarget(scoringTarget.previous())));
    operator.triangle().onTrue(Commands.runOnce(() -> selectAlgaePickup(algaePickup.next())));
    operator.cross().onTrue(Commands.runOnce(() -> selectAlgaePickup(algaePickup.previous())));

    climbOperator.cross().onTrue(climber.runClimber(ClimberConstants.kManualClimbPow));
    climbOperator.circle().onTrue(climber.runClimber(0));
    climbOperator.square().onTrue(climber.runClimber(-ClimberConstants.kManualClimbPow));
  }

  /** Raises the elevator first, then swings the arm out once it has some clearance. */
  private Command extendTo(double elevatorPos, double armPos) {
    return Commands.sequence(
        elevator.moveElevator(elevatorPos), Commands.waitSeconds(0.15), arm.moveArm(armPos));
  }

  private Command startIntake() {
    return Commands.sequence(
        intake.moveIntake(IntakeConstants.kIntakePos),
        intake.runIntake(IntakeConstants.kIntakePow),
        indexer.runIndexer(IndexerConstants.kIntakePow));
  }

  private Command stopIntakeRollers() {
    return Commands.sequence(
        intake.runIntake(IntakeConstants.kStopPow), indexer.runIndexer(IndexerConstants.kStopPow));
  }

  /** Reverses the rollers to push out extra coral, then stows the intake. */
  private Command ejectAndStowIntake(double ejectSeconds) {
    return Commands.sequence(
        intake.moveIntake(IntakeConstants.kParallelPos),
        intake.runIntake(IntakeConstants.kOuttakePow),
        indexer.runIndexer(IndexerConstants.kOuttakePow),
        Commands.waitSeconds(ejectSeconds),
        intake.moveIntake(IntakeConstants.kStowedPos),
        stopIntakeRollers());
  }

  private Command backOffFromReef() {
    return drivetrain
        .applyRequest(
            () ->
                robotCentricDrive
                    .withVelocityX(DriveConstants.kBackOffSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(0))
        .withTimeout(DriveConstants.kBackOffTime);
  }

  private Command setDriveSpeedMultiplier(double multiplier) {
    return Commands.runOnce(() -> driveSpeedMultiplier = multiplier);
  }

  private void selectScoringTarget(ScoringTarget target) {
    scoringTarget = target;
    publishSelections();
  }

  private void selectAlgaePickup(AlgaePickup pickup) {
    algaePickup = pickup;
    publishSelections();
  }

  private void publishSelections() {
    SmartDashboard.putString("Scoring Target", scoringTarget.name());
    SmartDashboard.putString("Algae Pickup", algaePickup.name());
  }
}
