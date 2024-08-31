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
    public void updateInputs(DriveIOInputs io) {
        io.frontLeftEncoderPosition = frontLeft.getEncoder().getPosition();
        io.frontLeftEncoderVelocity = frontLeft.getEncoder().getVelocity();

        io.frontRightEncoderPosition = frontRight.getEncoder().getPosition();
        io.frontRightEncoderVelocity = frontRight.getEncoder().getVelocity();

        io.rearLeftEncoderPosition = rearLeft.getEncoder().getPosition();
        io.rearLeftEncoderVelocity = rearLeft.getEncoder().getVelocity();

        io.rearRightEncoderPosition = rearRight.getEncoder().getPosition();
        io.rearRightEncoderVelocity = rearRight.getEncoder().getVelocity();
    }

    @Override
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        Logger.recordOutput("Drive/xSpeedCmd", xSpeed);
        Logger.recordOutput("Drive/ySpeedCmd",ySpeed);
        Logger.recordOutput("Drive/zRotation",zRotation);

        driveBase.driveCartesian(xSpeed, ySpeed, zRotation);
    }

}
