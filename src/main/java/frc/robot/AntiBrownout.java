package frc.robot;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;

public class AntiBrownout {
    public static AntiBrownout m_instance = new AntiBrownout();

    public static class PriorityMotor extends CANSparkFlex {
        private double m_speed;
        private int m_priority;
        private boolean m_update;

        public PriorityMotor(int deviceId, MotorType type) {
            super(deviceId, type);

            m_speed = 0;
            m_update = false;

            m_instance.register(this);
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
            if (!m_update) return;

            super.set(m_speed);
            m_priority = 0;
            m_update = false;
        }
    }

    final ArrayList<PriorityMotor> m_motors = new ArrayList<>();

    private AntiBrownout() {

    }

    private void register(PriorityMotor motor) {
        m_motors.add(motor);
        m_motors.sort((l, r) -> r.m_priority - l.m_priority);
    }

    public void postPeriodic() {
        m_motors.forEach((motor) -> motor.update());
    }
}