// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

    // Constants
    private static final double DEADZONE = 0.1;
    private static double speedMod = 1;
    private static final int TRIGGER = 1;
    private static final int THUMBBUTTON = 2;

    // Joysticks
    private final Joystick joyE = new Joystick(0);
    private final Joystick joyL = new Joystick(2);
    private final Joystick joyR = new Joystick(1);

    // Motor Controllers
    private final WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(9);
    private final WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(5);
    private final WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(2);
    private final WPI_VictorSPX backRightMotor = new WPI_VictorSPX(3);
    private final WPI_VictorSPX topCollectorMotor = new WPI_VictorSPX(6);
    private final WPI_VictorSPX bottomCollectorMotor = new WPI_VictorSPX(10);
    private final WPI_VictorSPX topArticulatingClimber = new WPI_VictorSPX(7);
    private final WPI_VictorSPX bottomArticulatingClimber = new WPI_VictorSPX(11);
    private final WPI_VictorSPX topExtendingClimber = new WPI_VictorSPX(4); // EXT
    private final WPI_VictorSPX bottomExtendingClimber = new WPI_VictorSPX(12); // EXT2
    // private final WPI_VictorSPX hoodMotor = new WPI_VictorSPX(8);
    private final CANSparkMax shooterMotor = new CANSparkMax(1, MotorType.kBrushless);

    // Motor Controller Groups
    private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);
    private final MotorControllerGroup extendingClimbers = new MotorControllerGroup(topExtendingClimber,
            bottomExtendingClimber);
    private final MotorControllerGroup articulatingClimbers = new MotorControllerGroup(topArticulatingClimber,
            bottomArticulatingClimber);

    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    double firstButtonTime = 0.0;
    double secondButtonTime = 0.0;
    double thirdButtonTime = 0.0;
    double teleopStartTime = 0.0;
    double robotTimer = Timer.getFPGATimestamp();

    @Override
    public void robotInit() {
        leftMotorGroup.setInverted(true);
        rightMotorGroup.setInverted(false);
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

        // Buttons
        boolean initialCollectionLButton = joyL.getRawButton(TRIGGER);
        boolean initialCollectionRButton = joyR.getRawButton(TRIGGER);
        boolean spinShooterButton = joyE.getRawButton(TRIGGER);

        boolean halfSpeedButtonL = joyL.getRawButton(THUMBBUTTON);
        boolean halfSpeedButtonR = joyR.getRawButton(THUMBBUTTON);
        boolean feederCollectionButton = joyE.getRawButton(THUMBBUTTON);

        boolean ejectButtonL = joyL.getRawButton(3);
        boolean ejectButtonR = joyR.getRawButton(3);

        boolean articulatingClimberButton = joyE.getRawButton(5);
        boolean articulatingClimberOtherwayButton = joyE.getRawButton(3);
        boolean windClimberButton = joyE.getRawButton(4);
        boolean unwindClimberButton = joyE.getRawButton(6);

        // Automated Climber Buttons, testing only, these button values are bad
        boolean firstButtonPressed = joyE.getRawButtonPressed(11);
        boolean secondButtonPressed = joyE.getRawButtonPressed(8);
        boolean thirdButtonPressed = joyE.getRawButtonPressed(7);

        // Constants
        double shooterSpeed = (joyE.getRawAxis(3) / 4) + 0.75; // converts [-1, 1] to [-1/4, 1/4] to [0.5, 1]
        double collectorSpeed = 0.5;
        double extendingClimberSpeed = 0.75;
        double articulatingClimberSpeed = 0.25;

        // Drivetrain Controls: left and right joysticks
        if (halfSpeedButtonL || halfSpeedButtonR) {
            speedMod = speedMod * 0.5;
        }

        if (Math.abs(joyL.getY()) > DEADZONE || Math.abs(joyR.getY()) > DEADZONE) {
            robotDrive.tankDrive(joyL.getY() * speedMod, joyR.getY() * speedMod);
        } else {
            robotDrive.tankDrive(0, 0);
        }

        // Shooting Motor: controlled by operator's trigger, speed set by variable flap
        shooterMotor.set(spinShooterButton ? shooterSpeed : 0);
        SmartDashboard.putNumber("Shooter Motor Percentage", shooterSpeed);

        // Collection Controls: driver's trigger + thumb button for intake/ejection,
        // operator's thumb button for feeding to spinning shooter wheel
        if (initialCollectionLButton || initialCollectionRButton) {
            topCollectorMotor.set(-collectorSpeed);
            bottomCollectorMotor.set(collectorSpeed);
        } else if (feederCollectionButton) {
            topCollectorMotor.set(collectorSpeed);
            bottomCollectorMotor.set(collectorSpeed);
        } else if (ejectButtonL || ejectButtonR) {
            topCollectorMotor.set(-collectorSpeed);
            bottomCollectorMotor.set(-collectorSpeed);
        } else {
            topCollectorMotor.set(0);
            bottomCollectorMotor.set(0);
        }

        // Automated Climber Controls: buttons 11, 8, 7
        /*
         * Everything is timed to be 1 second apart. Extending arms are lifted slightly
         * before they are rotated off of a bar.
         * -ExtendingClimberSpeed = winch pulls up robot
         * +ExtendingClimberSpeed = releases winch to allow extension of arms
         * -ArticulatingClimberSpeed = causes clockwise rotation |/
         * +ArticulatingClimberSpeed = causes counterclockwise rotation \|
         */
        if (firstButtonPressed) {
            // Button 11 is pressed to pull robot up onto bar 1
            firstButtonTime = Timer.getFPGATimestamp() - robotTimer;
            extendingClimbers.set(-extendingClimberSpeed);
            articulatingClimbers.set(0);

        } else if (firstButtonTime > 2 && secondButtonPressed) {
            secondButtonTime = Timer.getFPGATimestamp() - robotTimer;
            if (secondButtonTime <= 1) {
                // Button 8 is pressed to slightly raise extending arms off of bar 1
                // Articulating arms are rotated to angle towards bar 2
                extendingClimbers.set(extendingClimberSpeed * 0.5);
                articulatingClimbers.set(-articulatingClimberSpeed);
            } else if (secondButtonTime > 1 && secondButtonTime <= 2) {
                // After rotating, we extend to reach for bar 2
                extendingClimbers.set(extendingClimberSpeed);
                articulatingClimbers.set(0);
            } else if (secondButtonTime > 3 && secondButtonTime <= 4) {
                // After reaching towards bar 2, we rotate to allow extending arms to grasp it
                extendingClimbers.set(0);
                articulatingClimbers.set(articulatingClimberSpeed);
            }
        } else if (secondButtonTime > 3 && thirdButtonPressed) {
            thirdButtonTime = Timer.getFPGATimestamp() - robotTimer;
            if (thirdButtonTime < 1) {
                // Button 7 is pressed to pull robot onto bar 2
                extendingClimbers.set(-extendingClimberSpeed);
                articulatingClimbers.set(0);
            } else if (thirdButtonTime >= 1 && thirdButtonTime <= 2) {
                // Extending arms lower a little bit to allow articulating arms to sneak under
                // Articulating arms are rotated to vertical to allow us to grasp bar 2
                extendingClimbers.set(extendingClimberSpeed * 0.5);
                articulatingClimbers.set(articulatingClimberSpeed);
            } else if (thirdButtonTime > 2 && thirdButtonTime <= 3) {
                // Extending arms pull articulating arms up onto bar 2
                extendingClimbers.set(-extendingClimberSpeed);
                articulatingClimbers.set(0);
            } else if (thirdButtonTime > 3 && thirdButtonTime <= 4) {
                // Extending arms extend slightly so that they can rotate off of bar 2
                // Articulating arms rotate to angle towards bar 3
                extendingClimbers.set(extendingClimberSpeed * 0.5);
                articulatingClimbers.set(-articulatingClimberSpeed);
            } else if (thirdButtonTime > 4 && thirdButtonTime <= 5) {
                // Extending arms reach to bar 3
                extendingClimbers.set(extendingClimberSpeed);
                articulatingClimbers.set(0);
            } else if (thirdButtonTime > 5 && thirdButtonTime <= 6) {
                // Articulating arms rotate to allow extending arms to grab bar 3
                extendingClimbers.set(0);
                articulatingClimbers.set(articulatingClimberSpeed);
            } else if (thirdButtonTime > 6 && thirdButtonTime <= 7) {
                // Extending arms pull us on to bar 3
                // Articulating arms slowly rotate off of bar 2 to rest against bar 3
                // (so we don't get points deducted for touching bar 2)
                extendingClimbers.set(-extendingClimberSpeed);
                articulatingClimbers.set(articulatingClimberSpeed * 0.5);
            }
        } else {
            // manual time! please use manual please it's great
            // Articulating Climber Controls [MANUAL]: operator buttons 5 & 3
            if (articulatingClimberButton) {
                articulatingClimbers.set(articulatingClimberSpeed);
            } else if (articulatingClimberOtherwayButton) {
                articulatingClimbers.set(-articulatingClimberSpeed);
            } else {
                articulatingClimbers.set(0);
            }

            // Extending Climber Winch Controls [MANUAL]: operator buttons 6 & 4
            if (windClimberButton) {
                extendingClimbers.set(extendingClimberSpeed);
            } else if (unwindClimberButton) {
                extendingClimbers.set(-extendingClimberSpeed);
            } else {
                extendingClimbers.set(0);
            }

        }

    }

}