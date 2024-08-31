package frc.robot.subsystems.Drive;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
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
        inputs.frontLeftEncoderPosition = frontLeft.getEncoder().getPosition();
        inputs.frontLeftEncoderVelocity = frontLeft.getEncoder().getVelocity();
        inputs.frontLeftAppliedVolts = frontLeft.getAppliedOutput() * frontLeft.getBusVoltage();
        inputs.frontLeftCurrentAmps = frontLeft.getOutputCurrent();

        inputs.frontRightEncoderPosition = frontRight.getEncoder().getPosition();
        inputs.frontRightEncoderVelocity = frontRight.getEncoder().getVelocity();
        inputs.frontRightAppliedVolts = frontRight.getAppliedOutput() * frontRight.getBusVoltage();
        inputs.frontRightCurrentAmps = frontRight.getOutputCurrent();

        inputs.rearLeftEncoderPosition = rearLeft.getEncoder().getPosition();
        inputs.rearLeftEncoderVelocity = rearLeft.getEncoder().getVelocity();
        inputs.rearLeftAppliedVolts = rearLeft.getAppliedOutput() * rearLeft.getBusVoltage();
        inputs.rearLeftCurrentAmps = rearLeft.getOutputCurrent();

        inputs.rearRightEncoderPosition = rearRight.getEncoder().getPosition();
        inputs.rearRightEncoderVelocity = rearRight.getEncoder().getVelocity();
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
