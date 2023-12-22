package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveDistance extends CommandBase {

    public final Drivetrain drivetrain;
    public final double distance;
    
    public DrivetrainDriveDistance(Drivetrain drivetrain, double distance) {

        this.drivetrain = drivetrain;
        this.distance  = distance;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // runs 50 times per second
        drivetrain.tankDrive(2,2);
    
    }


    @Override
    public boolean isFinished() {
        return drivetrain.getLeftDistance() < Units.inchesToMeters(50) ;
    } 

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

}
