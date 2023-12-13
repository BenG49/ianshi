package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // 6 Motors

    private final DoubleSolenoid gearShift;

    private final CANSparkMax[] left;
    private final CANSparkMax[] right;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    
    private final AHRS navx;
    
    public Drivetrain() {
        CANSparkMax motor1 = new CANSparkMax(10, MotorType.kBrushless);
        CANSparkMax motor2 = new CANSparkMax(11, MotorType.kBrushless);
        CANSparkMax motor3 = new CANSparkMax(12, MotorType.kBrushless);
        CANSparkMax motor4 = new CANSparkMax(13, MotorType.kBrushless);
        CANSparkMax motor5 = new CANSparkMax(14, MotorType.kBrushless);
        CANSparkMax motor6 = new CANSparkMax(15, MotorType.kBrushless);
    
        left = new CANSparkMax[] {motor1, motor2,motor3};
        right = new CANSparkMax[] {motor4, motor5,motor6};
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(1,2);

        navx = new AHRS(I2C.Port.kMXP);

        gearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    }

    public void a() {
        gearShift.set(Value.kReverse);
    }

    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getLeftVoltage() {
        return left[0].getAppliedOutput();
    }

    public double getRightVoltage() {
    
        return right[0].getAppliedOutput();
    }

    public double getAngle() {
        return navx.getAngle();
    }
    // HIGH GEAR: Value.kReverse
    // LOW GEAR: Value.kForward

    // set all left motors to leftVoltage
    // set all right motors to rightVoltage
    public void tankDriveVolts(double leftVoltage, double rightVoltage) {
        for (CANSparkMax motor : left) {
            motor.setVoltage(leftVoltage);
        }
        for (CANSparkMax motor : right) {
            motor.setVoltage(rightVoltage);
        }
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Drivetrain/ NAVX Angle", getAngle());
        SmartDashboard.putNumber("Drivetrain/ Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/ Right Distance", getRightDistance());
        SmartDashboard.putNumber("Drivetrain/ Right Voltage", getRightVoltage());
        SmartDashboard.putNumber("Drivetrain/ Left Voltage", getLeftVoltage());
    }
    
}
