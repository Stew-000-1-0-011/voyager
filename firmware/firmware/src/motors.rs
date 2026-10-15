//! Apply robot-core bridge outputs to the timer PWM channels.

use embedded_hal_02::Pwm;
use robot_core::drive::bridge;
use stm32f1xx_hal::timer::Channel;

/// One DRV8871: IN1 / IN2 on two channels of the same timer.
pub fn drive<P: Pwm<Channel = Channel, Duty = u16>>(pwm: &mut P, in1: Channel, in2: Channel, cmd: i16) {
    let out = bridge(cmd, pwm.get_max_duty());
    pwm.set_duty(in1, out.in1);
    pwm.set_duty(in2, out.in2);
}
