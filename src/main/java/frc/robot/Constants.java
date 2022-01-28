// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

        // LED
        public static final int LED_STRIP_PORT = 0;
        public static final int NUMBER_OF_LEDS = 6;

        // Analog inputs
        public static final int RANGE_FINDER_PORT = 0;
        // Operator Interface
        public static final int DRIVER_REMOTE_PORT = 0;
        public static final int DRIVER_RIGHT_AXIS = 0; // Use 4 for Gamepad and 0 for Joystick
        public static final int DRIVER_LEFT_AXIS = 1;

        // Talons
        public static final int LEFT_TALON_LEADER = 5;
        public static final int RIGHT_TALON_LEADER = 3;

        public static final class DriveConstants {
                // Shaft Encoders attached to the RoboRIO
                public static final int[] kLeftEncoderPorts = new int[] { 0, 1 }; // DIO ports
                public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
                public static final boolean kLeftEncoderReversed = false;
                public static final boolean kRightEncoderReversed = true;

                public static final int kEncoderCPR = 1024;
                public static final double kWheelDiameterMeters = 0.15;
                public static final double kEncoderDistancePerPulse =
                                // Assumes the encoders are directly mounted on the wheel shafts
                                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

                public static final boolean kGyroReversed = true;

                // Constants for Simulation
                public static final double kTrackwidthMeters = 0.69;
                public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                                kTrackwidthMeters);

                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or
                // theoretically
                // for *your* robot's drive.
                // The Robot Characterization Toolsuite provides a convenient tool for obtaining
                // these
                // values for your robot.
                public static final double ksVolts = 0.22;
                public static final double kvVoltSecondsPerMeter = 1.98;
                public static final double kaVoltSecondsSquaredPerMeter = 0.2;

                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                // These characterization values MUST be determined either experimentally or
                // theoretically
                // for *your* robot's drive.
                // These two values are "angular" kV and kA
                public static final double kvVoltSecondsPerRadian = 1.5;
                public static final double kaVoltSecondsSquaredPerRadian = 0.3;

                // Used for Simulation
                public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
                                kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian,
                                kaVoltSecondsSquaredPerRadian);

                // Example values only -- use what's on your physical robot!
                public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
                public static final double kDriveGearing = 8;

        }

}
