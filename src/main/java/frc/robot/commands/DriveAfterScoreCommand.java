package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveAfterScoreCommand extends Command {
    private CommandSwerveDrivetrain drivebase;
    private SwerveRequest.RobotCentric drive 
    = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private Timer stopTimer;
    private double speed, seconds;

    public DriveAfterScoreCommand(CommandSwerveDrivetrain drivebase, double speed, double seconds) {
        this.drivebase = drivebase;
        this.speed = speed;
        this.seconds = seconds;
        addRequirements(drivebase);
    }
    
    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
    }

    @Override
    public void execute() {
        drivebase.setControl(
            drive.withVelocityX(speed)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(
            drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return stopTimer.hasElapsed(seconds);
    }
}