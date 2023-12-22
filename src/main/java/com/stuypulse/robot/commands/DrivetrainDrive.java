package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDrive extends CommandBase{


    public final Drivetrain drivetrain;
    private final Gamepad driver;
    double rightSpeed;
    double leftSpeed;


    public DrivetrainDrive(Drivetrain drivetrain, Gamepad driver) {

        this.drivetrain = drivetrain;
        this.driver = driver;
        addRequirements(drivetrain);

    }

    @Override
    public void execute() {

        rightSpeed = driver.getRightY() * 6;
        leftSpeed = driver.getLeftY() * 6;

        drivetrain.tankDriveVolts(leftSpeed, rightSpeed);
        
    }
    
}
