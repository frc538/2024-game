// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertNotNull;

/** Add your docs here. */
public class MecanumTest {
    MecanumDriveSubsystem mDriveSubsystem;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        mDriveSubsystem = new MecanumDriveSubsystem(); // create our drive
    }

    @AfterEach
    void tearDown() {
        mDriveSubsystem = null;
    }

    @Test
    void exists() {
        assertNotNull(mDriveSubsystem);
    }
}
