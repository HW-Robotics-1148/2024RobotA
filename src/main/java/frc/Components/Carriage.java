package frc.Components;

import java.sql.Driver;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Core.FieldData;
import frc.Devices.BinarySensor;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.MotionController;
import frc.lib.util.PDConstant;
import frc.lib.util.PIDController;
import frc.robot.team696.LED;

public class Carriage extends SubsystemBase {
    private boolean previousNote = false;
    public boolean justGotNote = false;
    public Double startPos = null;
    public TalonFX motor;
    public BinarySensor noteSensor;
    public MotionController controller = new PIDController(new PDConstant(4, 0));
    boolean prepShot = false;
    boolean isFiring = false;
    boolean hasNote = true;
    private double animSpeed = 0.0;

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public boolean isFiring() {
        return isFiring;
    }

    public Carriage(TalonFX motor, BinarySensor noteSensor) {
        this.motor = motor;
        this.noteSensor = noteSensor;
        motor.setVoltage(0);
        motor.getRawMotor().setNeutralMode(NeutralModeValue.Brake);
    }

    public void intake() {
        if (!hasNote) {
            motor.setVelocity(0.16 * 360);
        } else {

        }

    }

    public void resetMotor() {
        motor.resetEncoder();
    }

    public void runMotor() {
        motor.setVoltage(12);
        if (hasNote) {
            hasNote = false;
        }
    }

    public void stop() {
        motor.setVoltage(0);
    }

    public void intakeSlow() {
        motor.setVoltage(2);
    }

    public void outtakeSlow() {
        motor.setVoltage(-2);
    }

    public void outTake() {
        hasNote = false;
        motor.setVoltage(-12);
        startPos = null;
    }

    public void prepShot() {
        prepShot = true;
    }

    public void unPrepShot() {
        prepShot = false;
    }

    public void shoot() {
        Command shoot = new Command() {
            @Override
            public void initialize() {
                motor.setVoltage(12);
                startPos = null;
            }

            @Override
            public void execute() {
                motor.setVoltage(12);
            }

            @Override
            public void end(boolean interrupted) {
                motor.setVoltage(0);
                stop();
            }
        }.withTimeout(0.6);
        shoot.schedule();
        hasNote = false;

    }

    public boolean hasNote() {
        return hasNote;
    }

    @Override
    public void periodic() {

        if (noteSensor.justEnabled()) {
            hasNote = true;
            stop();
        }
        if (previousNote == false && hasNote == true) {
            justGotNote = true;
        } else {
            justGotNote = false;
        }

        SmartDashboard.putBoolean("Has Note", hasNote);
        if (!DriverStation.isDisabled()) {
            if (DriverStation.getMatchTime() < 20.0 && DriverStation.isTeleop()) {
                LED.get().setAnimation(new RainbowAnimation(1, 0.7, 33));
                // new LarsonAnimation(LED.get().redColor, 0, LED.get().blueColor, 255, 0.1, 25,
                // BounceMode.Center,
                // 1, 8)
            } else {
                if (hasNote) {
                    if (justGotNote) {
                        CommandScheduler.getInstance().schedule(new Command() {
                            @Override
                            public void initialize() {
                                LED.get()._candle.animate(new StrobeAnimation(0, 255, 0, 255, animSpeed, 32), 1);
                            }

                            public void execute() {
                                if (!hasNote) {
                                    LED.get()._candle.clearAnimation(1);
                                }
                            };

                            @Override
                            public void end(boolean interrupted) {
                                LED.get()._candle.clearAnimation(1);
                                LED.get().setColor(0, 255, 0);
                            }
                        }.withTimeout(4.0));
                    }
                } else {
                    LED.get().setColor(255, 0, 0);
                }
            }

        } else {
            if (FieldData.getIsRed()) {
                LED.get().setAnimation(new LarsonAnimation(255, 0, 0, 255, 0.4, 33, BounceMode.Front, 4));
            } else {
                LED.get().setAnimation(new LarsonAnimation(0, 0, 255, 255, 0.4, 33, BounceMode.Front, 4));
            }
        }
        // if (motor.getVoltage() >= 11) {
        // isFiring = true;
        // } else
        // isFiring = false;
        // }
        previousNote = hasNote;
    }
}
