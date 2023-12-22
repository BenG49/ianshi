package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveForever extends CommandBase {

    private final Drivetrain drivetrain;
    private final double speed;

    public DrivetrainDriveForever(Drivetrain drivetrain, double speed) { 

        this.drivetrain  = drivetrain;
        this.speed = speed;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // runs
        drivetrain.tankDriveVolts(speed, speed);
    }
    
}
