package frc.robot.subsystems.Drive;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;

public class DriveIOSparkMaxBrushed implements DriveIO {
    final CANSparkMax frontLeft;
    final CANSparkMax frontRight;
    final CANSparkMax rearLeft;
    final CANSparkMax rearRight;

    MecanumDrive driveBase;

    public DriveIOSparkMaxBrushed () {
        frontLeft = new CANSparkMax(Constants.CANIDs.FrontLeftDriveMotor, MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.CANIDs.FrontRightDriveMotor, MotorType.kBrushless);
        rearLeft = new CANSparkMax(Constants.CANIDs.RearLeftDriveMotor, MotorType.kBrushless);
        rearRight = new CANSparkMax(Constants.CANIDs.RearRighDriveMotor, MotorType.kBrushless);

        frontLeft.restoreFactoryDefaults();
        frontLeft.setInverted(false);
        frontLeft.burnFlash();

        rearLeft.restoreFactoryDefaults();
        rearLeft.setInverted(false);
        rearLeft.burnFlash();

        frontRight.restoreFactoryDefaults();
        frontRight.setInverted(true);
        frontRight.burnFlash();

        rearRight.restoreFactoryDefaults();
        rearRight.setInverted(true);
        rearRight.burnFlash();

        driveBase = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.frontLeftRad = frontLeft.getEncoder().getPosition() * 2 * Math.PI;
        inputs.frontLeftRadPerSec = frontLeft.getEncoder().getVelocity() * 2 * Math.PI * 60.0;
        inputs.frontLeftAppliedVolts = frontLeft.getAppliedOutput() * frontLeft.getBusVoltage();
        inputs.frontLeftCurrentAmps = frontLeft.getOutputCurrent();

        inputs.frontRightRad = frontRight.getEncoder().getPosition() * 2 * Math.PI;
        inputs.frontRightRadPerSec = frontRight.getEncoder().getVelocity() * 2 * Math.PI * 60.0;
        inputs.frontRightAppliedVolts = frontRight.getAppliedOutput() * frontRight.getBusVoltage();
        inputs.frontRightCurrentAmps = frontRight.getOutputCurrent();

        inputs.rearLeftRad = rearLeft.getEncoder().getPosition() * 2 * Math.PI;
        inputs.rearLeftRadPerSec = rearLeft.getEncoder().getVelocity() * 2 * Math.PI * 60.0;
        inputs.rearLeftAppliedVolts = rearLeft.getAppliedOutput() * rearLeft.getBusVoltage();
        inputs.rearLeftCurrentAmps = rearLeft.getOutputCurrent();

        inputs.rearRightRad = rearRight.getEncoder().getPosition() * 2 * Math.PI;
        inputs.rearRightRadPerSec = rearRight.getEncoder().getVelocity() * 2 * Math.PI * 60.0;
        inputs.rearRightAppliedVolts = rearRight.getAppliedOutput() * rearRight.getBusVoltage();
        inputs.rearRightCurrentAmps = rearRight.getOutputCurrent();
    }

    @Override
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        Logger.recordOutput("Drive/xSpeedCmd", xSpeed);
        Logger.recordOutput("Drive/ySpeedCmd",ySpeed);
        Logger.recordOutput("Drive/zRotation",zRotation);

        driveBase.driveCartesian(xSpeed, ySpeed, zRotation);
    }

}
