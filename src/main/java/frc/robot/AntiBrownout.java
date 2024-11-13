package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.AntiBrownoutConstants;
import frc.robot.Constants.AntiBrownoutConstants.RevBrushlessNeoConstants;

/**
 * Class for preventing persistent stage 1 brownouts and to allow critical robot motors to run remain running
 */
public class AntiBrownout {
    /**
     * Default instance of the AntiBrownout class
     * It is a very bad idea to create another instance
     */
    public static AntiBrownout m_defaultInstance = new AntiBrownout(true);

    /**
     * Used for accessing power distribution panel properties. Automatically detects what type of PDP is installed
     */
    private static PowerDistribution m_pdp = new PowerDistribution();

    /**
     * Internal variable for keeping track of enabled status
     */
    private static EnabledState m_enabled = EnabledState.STATE_DISABLED;

    /**
     * Enumerator for different enabled statuses
     */
    public enum EnabledState {
        /**
         * Do not run anti-brownout
         */
        STATE_DISABLED,

        /**
         * Run anti-brownout only during an actual brownout
         */
        STATE_ENABLED_SAFE,

        /**
         * Run anti-brownout during actual brownouts or when one is predicted
         */
        STATE_ENABLED_PREDICTIVE
    }

    /**
     * SAM type interface for passing motor profile information to AntiBrownout. This interface must be implemented for each motor with a different torque curve
     */
    public interface MotorProfile {
        /**
         * Should return the expected current draw at a certain set motor speed (from -1 to 1)
         * @param speed The speed of motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedCurrentDraw(double speed);

        /**
         * Should return the expected current draw at a certain set motor speed (from -1 to 1)
         * @param speed The speed of motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedVoltageDraw(double speed);
    }

    /**
     * This is a basic motor profile that will always lag behind by one period (hence reactive)
     */
    public static class ReactiveMotorProfile implements MotorProfile {
        /**
         * The PDP channel of the motor
         */
        private final int m_channel;

        /**
         * The type of motor
         */
        private final MotorType m_type;

        /**
         * Enumerator for various motor types
         */
        public enum MotorType {
            /**
             * REV Brushless Neo V1.1
             */
            TYPE_REV_BRUSHLESS_NEO11(RevBrushlessNeoConstants.kEmpiricalPeakPowerOutput);

            /**
             * The peak output power for the motor. Used for estimating voltage draw
             */
            private final double m_peakOutputPower;

            /**
             * Creates a new MotorType
             * @param peakOutputPower The peak output power of the motor in Watts
             */
            MotorType(double peakOutputPower) {
                m_peakOutputPower = peakOutputPower;
            }

            /**
             * Gets the peak power output of the motor
             * @return The peak power output of the motor in Watts
             */
            public double getPeakPowerOutput() {
                return m_peakOutputPower;
            }
        }

        /**
         * Creates a new ReactiveMotorProfile
         * @param channel The PDP channel of the motor
         * @param type The type of motor
         */
        public ReactiveMotorProfile(int channel, MotorType type) {
            m_channel = channel;
            m_type = type;
        }

        /**
         * Gets the expected current draw of the motor
         * @param speed The speed the motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedCurrentDraw(double speed) {
            return m_pdp.getCurrent(m_channel);
        }

        /**
         * Gets the expected voltage draw of the motor
         * @param speed The speed the motor from -1 to 1
         * @return The expected voltage draw in Volts
         */
        public double expectedVoltageDraw(double speed) {
            return Math.abs(speed) * m_type.getPeakPowerOutput() / expectedCurrentDraw(speed);
        }
    }

    /**
     * A CANSparkFlex subclass adding assignment priorities and brownout priorities
     */
    public static class PriorityMotor extends CANSparkFlex {
        /**
         * Internal setpoint of the motor
         */
        private double m_speed;

        /**
         * Internal highest priority of the setpoint
         */
        private int m_priority;

        /**
         * Whether the motor requires an update
         */
        private boolean m_update;

        /**
         * Profile information for the motor
         */
        private MotorProfile m_profile;

        /**
         * Creates a new PriorityMotor
         * @param deviceId The motor id
         * @param type The type of motor
         * @param profile The profile for the motor
         */
        public PriorityMotor(int deviceId, MotorType type, MotorProfile profile) {
            super(deviceId, type);

            m_speed = 0;
            m_update = false;
            m_profile = profile;

            m_defaultInstance.register(this);
        }

        @Override
        public void set(double speed) {
            set(speed, 0);
        }

        @Override
        public double get() {
            return m_speed;
        }

        /**
         * Sets the speed for the motor with a non-zero assignment priority
         * @param speed The speed to be set. Value should be between -1 and 1
         * @param priority The priority of the assignment
         */
        public void set(double speed, int priority) {
            if (priority >= m_priority) {
                m_speed = speed;
                m_priority = priority;
            }
            m_update = true;
        }

        /**
         * Internally used to sync m_speed with the superclass setpoint. Should only be called once each period, in postPeriodic
         */
        private void update() {
            if (!m_update)
                return;

            super.set(m_speed);
            m_priority = 0;
            m_update = false;
        }

        /**
         * Bypasses priority and sets setpoint immediatly. Should only be called once each period, in postPeriodic
         * @param speed
         */
        private void setUpdate(double speed) {
            m_speed = speed;
            update();
        }

        /**
         * Gets the expected current draw
         * @return The expected current draw in Amps
         */
        private double getExpectedCurrentDraw() {
            return m_profile.expectedCurrentDraw(m_speed);
        }

        /**
         * Gets the expected voltage draw
         * @return The expected voltage draw in Volts
         */
        private double getExpectedVoltageDraw() {
            return m_profile.expectedVoltageDraw(m_speed);
        }
    }

    /**
     * Used for keeping track of all registered motors in descending order of brownout priority
     */
    final ArrayList<PriorityMotor> m_motors = new ArrayList<>();

    /**
     * Creates a new AntiBrownout instance. Should only be called once
     * @param isDefault Whether the instance is the default instance. This is a fail safe in case of multiple instantiation
     */
    private AntiBrownout(boolean isDefault) {
        if (isDefault) {
            RobotController.setBrownoutVoltage(AntiBrownoutConstants.kStage1Thresh);
        }
    }

    /**
     * Registers a new motor to the AntiBrownout algorithm
     * @param motor The priority motor to register
     */
    private void register(PriorityMotor motor) {
        m_motors.add(motor);
        m_motors.sort((l, r) -> r.m_priority - l.m_priority);
    }

    /**
     * Runs before every periodic. Must be called from TimedRobot.robotPeriodic before the command scheduler call
     */
    public void prePeriodic() {
        //TODO: report all faults
    }

    /**
     * Runs after every periodic. Must be called from TimedRobot.robotPeriodic after the command scheduler call
     */
    public void postPeriodic() {
        switch (m_enabled) {
            case STATE_DISABLED:
                m_motors.forEach((motor) -> motor.update());
                break;
            case STATE_ENABLED_SAFE:
            case STATE_ENABLED_PREDICTIVE:
                final double pdpV = Robot.isReal() ? m_pdp.getVoltage() : 12.0; //TODO: simulate pdp
                final double effectiveV = m_enabled == EnabledState.STATE_ENABLED_SAFE ? pdpV : Math.min(pdpV, predictV());

                antiBrownout(effectiveV);
                break;
        }
    }

    /**
     * Calculates anti-brownout countermeasures. Can be overidden to use another algorithm.
     * @param effectiveV The effective voltage of the robot
     */
    protected void antiBrownout(double effectiveV) {
        if (effectiveV < AntiBrownoutConstants.kStage25Thresh) { // This does not really matter as Stage2 roborio has already disabled everything
            //TODO: implement log message and clear faults
        }

        else if (effectiveV < AntiBrownoutConstants.kStage2Thresh) { // Again, this does not really matter as Stage2 roborio has already disabled everything
            //TODO: implement log message and clear faults
        }

        else if (effectiveV < AntiBrownoutConstants.kStage1Thresh) {
            double budget = AntiBrownoutConstants.kMaxCurrentDraw;
            for (PriorityMotor motor : m_motors) {
                //TODO: implement better scalling
                motor.setUpdate(stage1Scaling(budget, motor.m_priority) * motor.m_speed);
                budget -= motor.getExpectedCurrentDraw();
            }
        }

        else {
           //TODO: implement log message and clear faults
        }
    }

    /**
     * Calculates the scaling during stage 1 brownout. Can be overriden to use another algorithm.
     * @param budget The current budget in Amps
     * @param priority The motors priority on a zero based scale
     * @return The scaling value for the motor speed. Will always be between 0 and 1
     */
    protected double stage1Scaling(double budget, double priority) {
        return 1; //TODO: implement an actual scaling function
    }

    /**
     * Predicts the total voltage draw
     * @return The total voltage draw in Amps
     */
    public double predictV() {
        double voltageDraw = 0;
        for (PriorityMotor motor : m_motors) {
            voltageDraw += motor.getExpectedVoltageDraw();
        }
        return voltageDraw;
    }

    /**
     * Changes the enabled status
     * @param enabled The enabled status to be set to
     */
    public void setEnabled(EnabledState enabled) {
        m_enabled = enabled;
    }
}