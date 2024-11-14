package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.AntiBrownoutConstants;
import frc.robot.Constants.AntiBrownoutConstants.RevBrushlessNeoConstants;

/**
 * Class for preventing persistent stage 1 brownouts and to allow critical robot
 * motors to run remain running
 */
public class AntiBrownout {
    /**
     * Default instance of the AntiBrownout class
     * It is a very bad idea to create another instance
     */
    public static final AntiBrownout m_defaultInstance = new AntiBrownout();

    /**
     * Used for accessing power distribution panel properties. Automatically detects
     * what type of PDP is installed
     */
    private static final PowerDistribution m_pdp = new PowerDistribution();

    /**
     * Internal variable for keeping track of enabled status
     */
    private static EnabledState m_enabled = EnabledState.STATE_DISABLED;

    /**
     * Used to tracking the current Anti-Brownout strategy
     */
    private static Strategy m_strategy = GenericStrategy.STRATEGY_NONE;

    /**
     * The highest motor priority
     */
    private static int m_base = -1;

    /**
     * Number of motor priorities
     */
    private static int m_numP = 0;

    /**
     * Enumerator for different enabled statuses
     */
    public static enum EnabledState {
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
        STATE_ENABLED_PREDICTIVE;
    }

    /**
     * SAM type interface for passing motor profile information to AntiBrownout.
     * This interface must be implemented for each motor with a different torque
     * curve
     */
    public static interface MotorProfile {
        /**
         * Should return the expected current draw at a certain set motor speed (from -1
         * to 1)
         * 
         * @param speed The speed of motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedCurrentDraw(double speed);

        /**
         * Should return the expected current draw at a certain set motor speed (from -1
         * to 1)
         * 
         * @param speed The speed of motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedVoltageDraw(double speed);
    }

    /**
     * This is a basic motor profile that will always lag behind by one period
     * (hence reactive)
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
             * 
             * @param peakOutputPower The peak output power of the motor in Watts
             */
            MotorType(double peakOutputPower) {
                m_peakOutputPower = peakOutputPower;
            }

            /**
             * Gets the peak power output of the motor
             * 
             * @return The peak power output of the motor in Watts
             */
            public double getPeakPowerOutput() {
                return m_peakOutputPower;
            }
        }

        /**
         * Creates a new ReactiveMotorProfile
         * 
         * @param channel The PDP channel of the motor
         * @param type    The type of motor
         */
        public ReactiveMotorProfile(int channel, MotorType type) {
            m_channel = channel;
            m_type = type;
        }

        /**
         * Gets the expected current draw of the motor
         * 
         * @param speed The speed the motor from -1 to 1
         * @return The expected current draw in Amps
         */
        public double expectedCurrentDraw(double speed) {
            return m_pdp.getCurrent(m_channel);
        }

        /**
         * Gets the expected voltage draw of the motor
         * 
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
         * 
         * @param deviceId The motor id
         * @param type     The type of motor
         * @param profile  The profile for the motor
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
         * 
         * @param speed    The speed to be set. Value should be between -1 and 1
         * @param priority The positive priority of the assignment
         */
        public void set(double speed, int priority) {
            if (priority >= m_priority) {
                m_speed = speed;
                m_priority = Math.abs(priority);
            }
            m_update = true;
        }

        /**
         * Internally used to sync m_speed with the superclass setpoint. Should only be
         * called once each period, in postPeriodic
         */
        private void update() {
            if (!m_update)
                return;

            super.set(m_speed);
            m_priority = 0;
            m_update = false;
        }

        /**
         * Bypasses priority and sets setpoint immediatly. Should only be called once
         * each period, in postPeriodic
         * 
         * @param speed
         */
        private void setUpdate(double speed) {
            m_speed = speed;
            update();
        }

        /**
         * Gets the expected current draw
         * 
         * @return The expected current draw in Amps
         */
        private double getExpectedCurrentDraw() {
            return m_profile.expectedCurrentDraw(m_speed);
        }

        /**
         * Gets the expected voltage draw
         * 
         * @return The expected voltage draw in Volts
         */
        private double getExpectedVoltageDraw() {
            return m_profile.expectedVoltageDraw(m_speed);
        }
    }

    /**
     * Interface for creating custom strategies
     */
    public static interface Strategy {
        /**
         * Algorithm used for scaling during stage1 brownout
         * 
         * @param priority Motor priority
         * @param base     Highest priority
         * @param draw     Expected motor current draw in Amps
         * @param netDraw  Total motor current draw for all motors with priority
         *                 {@code priority} in Amps
         * @param budget   Remaining current budget in Amps
         * @return The scaling factor for the motor speed. A value between 0 and 1
         */
        public double stage1Scaling(int priority, int base, double draw, double netDraw, double budget, double speed);

        /**
         * Checks if strategy is a parallel strategy (all motors in a priority group are
         * considers simulataniously)
         * 
         * @return true if the strategy is parallel
         */
        public boolean isParallel();
    }

    /**
     * A generic strategy enumerator
     */
    public static enum GenericStrategy implements Strategy {
        /**
         * No scaling strategy. All motor speeds remain unchanged
         */
        STRATEGY_NONE((priority, base, budget) -> 1d),

        /**
         * Scaling strategy that considers each motor individually
         */
        STRATEGY_SEQUENTIAL((priority, base, budget) -> priority / base),

        /**
         * Scaling strategy that considers each motor within its priority group
         */
        STRATEGY_PARALLEL(
                (priority, base, budget) -> (priority * AntiBrownoutConstants.kMaxCurrentDraw) / (base * budget));

        /**
         * SAM type with double return used to store scaling algorithm
         */
        private final Scaling m_scaling;

        /**
         * SAM type interface with double return used to store scaling algorithms
         */
        private static interface Scaling {
            public double scale(double priority, double base, double budget);
        }

        /**
         * Constructs a new enumerator instance
         * 
         * @param scaling The scaling strategy to use
         */
        private GenericStrategy(Scaling scaling) {
            m_scaling = scaling;
        }

        @Override
        public double stage1Scaling(int priority, int base, double draw, double netDraw, double budget, double speed) {
            return m_scaling.scale(priority, base, budget);
        }

        @Override
        public boolean isParallel() {
            switch (this) {
                case STRATEGY_NONE:
                case STRATEGY_SEQUENTIAL:
                    return false;
                case STRATEGY_PARALLEL:
                    return true;
                default:
                    return false;
            }
        }
    }

    /**
     * Used for keeping track of all registered motors in descending order of
     * brownout priority
     */
    final ArrayList<PriorityMotor> m_motors = new ArrayList<>();

    /**
     * Creates a new AntiBrownout instance. Should only be called once
     * 
     * @param isDefault Whether the instance is the default instance. This is a fail
     *                  safe in case of multiple instantiation
     */
    private AntiBrownout() {
        RobotController.setBrownoutVoltage(AntiBrownoutConstants.kStage1Thresh);
    }

    /**
     * Registers a new motor to the AntiBrownout algorithm
     * 
     * @param motor The priority motor to register
     */
    private void register(PriorityMotor motor) {
        m_motors.add(motor);
        m_motors.sort((l, r) -> r.m_priority - l.m_priority);

        if (motor.m_priority > m_base) {
            m_numP++;
            m_base = motor.m_priority;
        }
    }

    /**
     * Runs before every periodic. Must be called from TimedRobot.robotPeriodic
     * before the command scheduler call
     */
    public void prePeriodic() {
        // TODO: report all faults
    }

    /**
     * Runs after every periodic. Must be called from TimedRobot.robotPeriodic after
     * the command scheduler call
     */
    public void postPeriodic() {
        switch (m_enabled) {
            case STATE_DISABLED:
                m_motors.forEach((motor) -> motor.update());
                break;
            case STATE_ENABLED_SAFE:
            case STATE_ENABLED_PREDICTIVE:
                final double pdpV = Robot.isReal() ? m_pdp.getVoltage() : 12.0; // TODO: simulate pdp
                final double effectiveV = m_enabled == EnabledState.STATE_ENABLED_SAFE ? pdpV
                        : Math.min(pdpV, predictV());

                antiBrownout(effectiveV);
                break;
        }
    }

    /**
     * Calculates anti-brownout countermeasures. Can be overidden to use another
     * algorithm.
     * 
     * @param effectiveV The effective voltage of the robot
     */
    protected void antiBrownout(double effectiveV) {
        if (effectiveV < AntiBrownoutConstants.kStage25Thresh) { // This does not really matter as Stage2 roborio has
                                                                 // already disabled everything
            // TODO: implement log message and clear faults
        }

        else if (effectiveV < AntiBrownoutConstants.kStage2Thresh) { // Again, this does not really matter as Stage2
                                                                     // roborio has already disabled everything
            // TODO: implement log message and clear faults
        }

        else if (effectiveV < AntiBrownoutConstants.kStage1Thresh) {
            double budget = AntiBrownoutConstants.kMaxCurrentDraw;

            if (m_strategy.isParallel()) {
                final double[] priorityNets = new double[m_numP];
                final double[] priorityBudgets = new double[m_numP];
                double netDraw = 0;
                int lastP = m_base;
                int i = 0;

                priorityBudgets[0] = budget;

                for (PriorityMotor motor : m_motors) {
                    netDraw += motor.getExpectedCurrentDraw(); // TODO: factor in the fact that netDraw will be lower
                                                               // after scaling (which will in turn increase budget in
                                                               // future priorities)

                    if (lastP != motor.m_priority) {
                        budget -= netDraw;
                        priorityNets[i] = netDraw;
                        priorityBudgets[i + 1] = budget;

                        netDraw = 0;
                        lastP = motor.m_priority;
                        i++;
                    }
                }
                priorityNets[i] = netDraw;

                lastP = m_base;
                i = 0;

                for (PriorityMotor motor : m_motors) {
                    if (lastP != motor.m_priority) {
                        i++;
                        lastP = motor.m_priority;
                    }

                    motor.setUpdate(motor.m_speed * m_strategy.stage1Scaling(motor.m_priority, m_base,
                            motor.getExpectedCurrentDraw(), priorityNets[i], priorityBudgets[i], motor.m_speed));
                }
            } else {
                for (PriorityMotor motor : m_motors) {
                    motor.setUpdate(motor.m_speed * m_strategy.stage1Scaling(motor.m_priority, m_base,
                            motor.getExpectedCurrentDraw(), motor.getExpectedCurrentDraw(), budget, motor.m_speed));
                    budget -= motor.getExpectedCurrentDraw();
                }
            }
        }

        else {
            // TODO: implement log message and clear faults
        }
    }

    /**
     * Predicts the total voltage draw
     * 
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
     * 
     * @param enabled The enabled status to be set to
     */
    public void setEnabled(EnabledState enabled) {
        m_enabled = enabled;
    }

    public void setStrategy(Strategy strategy) {
        m_strategy = strategy;
    }
}
