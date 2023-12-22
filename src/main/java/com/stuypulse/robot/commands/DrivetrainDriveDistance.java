package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveDistance extends CommandBase {

    public final Drivetrain drivetrain;
    public final double distance;
    
    public DrivetrainDriveDistance(Drivetrain drivetrain, double distance) {

        this.drivetrain = drivetrain;
        this.distance  = distance;

    }

    @Override
    public void execute() {
        // runs 50 times per second
        drivetrain.tankDriveVolts(1, 1);
    
    }


    @Override
    public boolean isFinished() {
        if (drivetrain.getLeftDistance() < 50) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

}
