package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // 6 Motors

    private final DoubleSolenoid gearShift;

    private final CANSparkMax[] left;
    private final CANSparkMax[] right;

    private final DifferentialDrive drivetrain;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    
    private final AHRS navx;

    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    public Drivetrain() {
        CANSparkMax motor1 = new CANSparkMax(10, MotorType.kBrushless);
        CANSparkMax motor2 = new CANSparkMax(11, MotorType.kBrushless);
        CANSparkMax motor3 = new CANSparkMax(12, MotorType.kBrushless);
        CANSparkMax motor4 = new CANSparkMax(13, MotorType.kBrushless);
        CANSparkMax motor5 = new CANSparkMax(14, MotorType.kBrushless);
        CANSparkMax motor6 = new CANSparkMax(15, MotorType.kBrushless);
    
        left = new CANSparkMax[] {motor1, motor2,motor3};
        right = new CANSparkMax[] {motor4, motor5,motor6};

        drivetrain = new DifferentialDrive(
            new MotorControllerGroup(left),
            new MotorControllerGroup(right)
        );

        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(1,2);

        navx = new AHRS(I2C.Port.kMXP);

        gearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
        odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 0, 0, getPose());
        field = new Field2d();
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

        drivetrain.feed();  
    }

    private void updateOdometry() {
        odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
    }

    private void reset(Pose2d location) {
        leftEncoder.reset();
        rightEncoder.reset();
        navx.reset();
        odometry.resetPosition(navx.getRotation2d(), 0, 0, location);
    }

    private void reset() {
        reset(new Pose2d());
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    public Field2d getField() {
        return field;
    }

    //drive commands
    public void stop() {
        drivetrain.stopMotor();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        drivetrain.curvatureDrive(speed, rotation, isQuickTurn);
    }

    @Override 
    public void periodic() {
        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Field", getField());

        SmartDashboard.putNumber("Drivetrain/ NAVX Angle", getAngle());
        SmartDashboard.putNumber("Drivetrain/ Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Drivetrain/ Right Distance", getRightDistance());
        SmartDashboard.putNumber("Drivetrain/ Right Voltage", getRightVoltage());
        SmartDashboard.putNumber("Drivetrain/ Left Voltage", getLeftVoltage());
    }
    
}
