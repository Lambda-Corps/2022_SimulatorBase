// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
        public static final int LEFT_TALON_LEADER = 1;
        public static final int LEFT_TALON_FOLLOWER = 3;
        public static final int RIGHT_TALON_LEADER = 2;
        public static final int RIGHT_TALON_FOLLOWER = 4;

        public static final class DriveConstants {
                /*
                * These numbers are an example AndyMark Drivetrain with some additional weight.
                * This is a fairly light robot.
                * Note you can utilize results from robot characterization instead of
                * theoretical numbers.
                * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
                * characterization/introduction.html#introduction-to-robot-characterization
                */
                public static final int kCountsPerRev = 2048;    // Encoder counts per revolution of the motor shaft.
                public static final double kSensorGearRatio = 10.71; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
                                                                 // drivetrain, encoders mount 1:1 with the gearbox shaft.
                public static final double kGearRatio = 10.71;   // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
                                                                 // of on the gearbox.
                public static final double kWheelRadiusInches = 3.25;
                public static final int k100msPerSecond = 10;
                public static final double kNeutralDeadband = 0.003;

                /**
                  * PID Gains may have to be adjusted based on the responsiveness of control loop.
                  * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
                  * Not all set of Gains are used in this project and may be removed as desired.
                  * 
                  * 	                                    	        kP   kI   kD   kF             Iz    PeakOut */
                public final static Gains kGains_Turning = new Gains( 0.02, 0.0, 0.0, 0.0,            200,  1.00 );
                
                /** ---- Flat constants, you should not need to change these ---- */
                /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
                public final static int REMOTE_0 = 0;
                public final static int REMOTE_1 = 1;
                /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
                public final static int PID_PRIMARY = 0;
                public final static int PID_TURN = 1;
                /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
                public final static int SLOT_0 = 0;
                public final static int SLOT_1 = 1;
                public final static int SLOT_2 = 2;
                public final static int SLOT_3 = 3;
                /* ---- Named slots, used to clarify code ---- */
                public final static int kSlot_Distanc = SLOT_0;
                public final static int kSlot_Turning = SLOT_1;
                public final static int kSlot_Velocit = SLOT_2;
                public final static int kSlot_MotProf = SLOT_3;
                
                public static boolean kGyroReversed = true;
                public static int kTurnTravelUnitsPerRotation = 3600;
                public static int kEncoderUnitsPerRotation = 88554;

                /* ---- Characterization Calculations ---- */
                public static final double ksVolts = 0.65154;
                public static final double kvVoltSecondsPerMeter = .000012616;
                public static final double kaVoltSecondsSquaredPerMeter = 0.0000003799;
                public static final double kPDriveVel = 8.5;
                public static final double kTrackwidthMeters = 0.6731;
                public static final DifferentialDriveKinematics kDriveKinematics = 
                        new DifferentialDriveKinematics(kTrackwidthMeters);
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
                public static final double kRamseteB = 2;
                public static final double kRamseteZeta = 0.7;
        }
}
