package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.AntiBrownoutConstants;
import frc.robot.Constants.AntiBrownoutConstants.RevBrushlessNeoConstants;

public class AntiBrownout {
    public static AntiBrownout m_defaultInstance = new AntiBrownout(true);

    private static PowerDistribution m_pdp = new PowerDistribution();
    private static EnabledState m_enabled = EnabledState.STATE_DISABLED;

    public enum EnabledState {
        STATE_DISABLED,
        STATE_ENABLED_SAFE,
        STATE_ENABLED_PREDICTIVE
    }

    public interface MotorProfile {
        public double expectedCurrentDraw(double speed);
    }

    public static class ReactiveMotorProfile implements MotorProfile {
        private final int m_channel;
        private final MotorType m_type;

        public enum MotorType {
            TYPE_REV_BRUSHLESS_NEO(RevBrushlessNeoConstants.kEmpiricalPeakPowerOutput);

            private final double m_peakOutputPower;

            MotorType(double peakOutputPower) {
                m_peakOutputPower = peakOutputPower;
            }

            public double getPeakPowerOutput() {
                return m_peakOutputPower;
            }
        }

        public ReactiveMotorProfile(int channel, MotorType type) {
            m_channel = channel;
            m_type = type;
        }

        public double expectedCurrentDraw(double speed) {
            return m_pdp.getCurrent(m_channel);
        }

        public double expectedVoltageDraw(double speed) {
            return Math.abs(speed) * m_type.getPeakPowerOutput() / expectedCurrentDraw(speed);
        }
    }

    public static class PriorityMotor extends CANSparkFlex {
        private double m_speed;
        private int m_priority;
        private boolean m_update;
        private MotorProfile m_profile;

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

        public void set(double speed, int priority) {
            if (priority >= m_priority) {
                m_speed = speed;
                m_priority = priority;
            }
            m_update = true;
        }

        private void update() {
            if (!m_update)
                return;

            super.set(m_speed);
            m_priority = 0;
            m_update = false;
        }

        private void setUpdate(double speed) {
            m_speed = speed;
            update();
        }

        private double getExpectedCurrentDraw() {
            return m_profile.expectedCurrentDraw(m_speed);
        }

        private double getExpectedVoltageDraw() {
            return m_profile.expectedCurrentDraw(m_speed);
        }
    }

    final ArrayList<PriorityMotor> m_motors = new ArrayList<>();

    private AntiBrownout(boolean isDefault) {
        if (isDefault) {
            RobotController.setBrownoutVoltage(AntiBrownoutConstants.kStage1Thresh);
        }
    }

    private void register(PriorityMotor motor) {
        m_motors.add(motor);
        m_motors.sort((l, r) -> r.m_priority - l.m_priority);
    }

    public void prePeriodic() {
        //TODO: report all faults
    }

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

    protected double stage1Scaling(double budget, double priority) {
        return 0; //TODO: implement an actual scaling function
    }

    public double predictV() {
        double voltageDraw = 0;
        for (PriorityMotor motor : m_motors) {
            voltageDraw += motor.getExpectedVoltageDraw();
        }
        return voltageDraw;
    }

    public void setEnabled(EnabledState enabled) {
        m_enabled = enabled;
    }
}