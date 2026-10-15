//! Carrier robot firmware: STM32F103C8 (Bluepill), RTICX.
//!
//! Pin assignment: spec section 5.4. All decisions live in robot-core; this crate
//! only moves data between peripherals and `Controller::step`.

#![no_std]
#![no_main]

mod motors;
mod sonar;

use defmt_rtt as _;
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;

systick_monotonic!(Mono, 1000);

pub const CPU_HZ: u32 = 72_000_000;

#[rticx_cortex_m::app(device = stm32f1xx_hal::pac, dispatchers = [SPI1, SPI2])]
mod app {
    use super::*;
    use crate::motors::drive;
    use crate::sonar::{ECHO_BITS, EchoCapture};
    use cortex_m::peripheral::DWT;
    use robot_core::control::{Controller, Inputs};
    use robot_core::gripper::DEFAULT_GRIPPER;
    use robot_core::protocol::{Command, FrameDecoder, MAX_FRAME, Telemetry};
    use robot_core::quad::QuadDecoder;
    use robot_core::safety::adc_to_mv;
    use stm32f1xx_hal::{
        adc::Adc,
        gpio::{Alternate, Analog, Edge, ExtiPin, Output, PA4, PA8, PA9, PB0, PB1, PB6, PB7, PB8, PB9, PC13, PushPull},
        pac::{self, ADC1, TIM1, TIM2, TIM3, TIM4},
        prelude::*,
        rcc,
        timer::{
            Ch, Channel, PwmHz, Tim1NoRemap, Tim4NoRemap,
            pwm_input::{Qei, QeiExt, QeiOptions},
        },
        usb::{Peripheral, UsbBus, UsbBusType},
        watchdog::IndependentWatchdog,
    };
    use usb_device::{bus::UsbBusAllocator, prelude::*};
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    type DrivePwm = PwmHz<TIM4, Tim4NoRemap, (Ch<0>, Ch<1>, Ch<2>, Ch<3>),
        (PB6<Alternate>, PB7<Alternate>, PB8<Alternate>, PB9<Alternate>)>;
    type GripPwm = PwmHz<TIM1, Tim1NoRemap, (Ch<0>, Ch<1>), (PA8<Alternate>, PA9<Alternate>)>;

    pub struct Usb {
        dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
    }

    #[shared]
    struct Shared {
        usb: Usb,
        latest_cmd: Option<Command>,
        grip_quad: QuadDecoder,
        sonar_mm: [u16; 4],
    }

    static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

    #[init]
    fn system_init() -> (Shared, TaskInits) {
        let dp = unsafe { pac::Peripherals::steal() };
        let mut cp = unsafe { cortex_m::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.freeze(
            rcc::Config::hse(8.MHz()).sysclk(72.MHz()).pclk1(36.MHz()).adcclk(12.MHz()),
            &mut flash.acr,
        );
        let mut afio = dp.AFIO.constrain(&mut rcc);
        let mut gpioa = dp.GPIOA.split(&mut rcc);
        let mut gpiob = dp.GPIOB.split(&mut rcc);
        let mut gpioc = dp.GPIOC.split(&mut rcc);
        let mut exti = dp.EXTI;

        Mono::start(cp.SYST, CPU_HZ);
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();

        // PB3 / PB4 are JTAG pins: keep SWD only
        let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // drive PWM: TIM4 CH1..4 = PB6..PB9 (L_IN1, L_IN2, R_IN1, R_IN2)
        let drive_pins = (
            gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl),
            gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh),
        );
        let mut drive_pwm = dp.TIM4.pwm_hz::<Tim4NoRemap, _, _>(drive_pins, &mut afio.mapr, 20.kHz(), &mut rcc);
        // gripper PWM: TIM1 CH1..2 = PA8, PA9 (G_IN1, G_IN2)
        let grip_pins = (
            gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
        );
        let mut grip_pwm = dp.TIM1.pwm_hz::<Tim1NoRemap, _, _>(grip_pins, &mut afio.mapr, 20.kHz(), &mut rcc);
        for ch in [Channel::C1, Channel::C2, Channel::C3, Channel::C4] {
            drive_pwm.enable(ch);
        }
        grip_pwm.enable(Channel::C1);
        grip_pwm.enable(Channel::C2);
        // start braked
        drive(&mut drive_pwm, Channel::C1, Channel::C2, 0);
        drive(&mut drive_pwm, Channel::C3, Channel::C4, 0);
        drive(&mut grip_pwm, Channel::C1, Channel::C2, 0);

        // wheel encoders
        let qei_left = dp.TIM2.qei((gpioa.pa0, gpioa.pa1), QeiOptions::default(), &mut rcc);
        let qei_right = dp.TIM3.qei((gpioa.pa6, gpioa.pa7), QeiOptions::default(), &mut rcc);

        // gripper encoder: EXTI3 / EXTI4, both edges
        let mut grip_a = pb3.into_pull_up_input(&mut gpiob.crl);
        let mut grip_b = pb4.into_pull_up_input(&mut gpiob.crl);
        for p in [&mut grip_a as &mut dyn ExtiPin, &mut grip_b] {
            p.make_interrupt_source(&mut afio);
            p.trigger_on_edge(&mut exti, Edge::RisingFalling);
            p.enable_interrupt(&mut exti);
        }

        // ultrasonic: triggers PB0 / PB1, echoes PB12..PB15 on EXTI15_10
        let trig_front = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
        let trig_rear = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        let mut e12 = gpiob.pb12.into_floating_input(&mut gpiob.crh);
        let mut e13 = gpiob.pb13.into_floating_input(&mut gpiob.crh);
        let mut e14 = gpiob.pb14.into_floating_input(&mut gpiob.crh);
        let mut e15 = gpiob.pb15.into_floating_input(&mut gpiob.crh);
        for p in [&mut e12 as &mut dyn ExtiPin, &mut e13, &mut e14, &mut e15] {
            p.make_interrupt_source(&mut afio);
            p.trigger_on_edge(&mut exti, Edge::RisingFalling);
            p.enable_interrupt(&mut exti);
        }

        // battery voltage on PA4
        let adc = Adc::new(dp.ADC1, &mut rcc);
        let vbat = gpioa.pa4.into_analog(&mut gpioa.crl);

        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // USB: pull D+ low briefly so the host re-enumerates after a reset
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        cortex_m::asm::delay(CPU_HZ / 100);
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);
        let bus = unsafe {
            let p = &raw mut USB_BUS;
            *p = Some(UsbBus::new(Peripheral { usb: dp.USB, pin_dm: gpioa.pa11, pin_dp: usb_dp }));
            (*p).as_ref().unwrap()
        };
        let serial = SerialPort::new(bus);
        let dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x16C0, 0x27DD))
            .strings(&[StringDescriptors::default()
                .manufacturer("voyager")
                .product("voyager-carrier")
                .serial_number("0001")])
            .unwrap()
            .device_class(USB_CLASS_CDC)
            .build();

        // independent watchdog: if the control loop stops (hang, panic, fault) the MCU resets within
        // 100ms; after reset the PWM pins float and the DRV8871 input pull-downs let the motors coast
        let mut iwdg = IndependentWatchdog::new(dp.IWDG);
        iwdg.start(100.millis());

        defmt::info!("init done");
        (
            Shared { usb: Usb { dev, serial }, latest_cmd: None, grip_quad: QuadDecoder::default(), sonar_mm: [0; 4] },
            TaskInits {
                usb_rx: UsbRx { decoder: FrameDecoder::new() },
                usb_tx: UsbTx,
                grip_enc_a: GripEncA,
                grip_enc_b: GripEncB,
                sonar_echo: SonarEcho { capture: EchoCapture::default() },
                control: Control {
                    drive_pwm,
                    grip_pwm,
                    qei_left,
                    qei_right,
                    adc,
                    vbat,
                    led,
                    iwdg,
                    controller: Controller::new(DEFAULT_GRIPPER),
                },
                sonar_trigger: SonarTrigger { front: trig_front, rear: trig_rear },
            },
        )
    }

    #[post_init]
    fn post_init() {
        let _ = Control::spawn(());
        let _ = SonarTrigger::spawn(());
    }

    fn poll_usb(usb: &mut Usb, decoder: &mut FrameDecoder, latest: &mut Option<Command>) {
        if !usb.dev.poll(&mut [&mut usb.serial]) {
            return;
        }
        let mut buf = [0u8; 64];
        while let Ok(n) = usb.serial.read(&mut buf) {
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                if let Some(payload) = decoder.push(b) {
                    if let Ok(c) = Command::decode(payload) {
                        *latest = Some(c);
                    }
                }
            }
        }
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 3, shared = [usb, latest_cmd])]
    struct UsbRx {
        decoder: FrameDecoder,
    }
    impl RticTask for UsbRx {
        fn exec(&mut self) {
            let mut decoder = core::mem::take(&mut self.decoder);
            let shared = self.shared();
            let (mut usb, mut latest) = (shared.usb, shared.latest_cmd);
            usb.lock(|usb| latest.lock(|latest| poll_usb(usb, &mut decoder, latest)));
            self.decoder = decoder;
        }
    }

    #[task(binds = USB_HP_CAN_TX, priority = 3, shared = [usb])]
    struct UsbTx;
    impl RticTask for UsbTx {
        fn exec(&mut self) {
            self.shared().usb.lock(|usb| {
                usb.dev.poll(&mut [&mut usb.serial]);
            });
        }
    }

    fn grip_edge(quad: &mut QuadDecoder, line: u8) {
        let exti = unsafe { &*pac::EXTI::ptr() };
        exti.pr().write(|w| unsafe { w.bits(1 << line) });
        let idr = unsafe { (*pac::GPIOB::ptr()).idr().read().bits() };
        quad.update(idr & (1 << 3) != 0, idr & (1 << 4) != 0);
    }

    #[task(binds = EXTI3, priority = 4, shared = [grip_quad])]
    struct GripEncA;
    impl RticTask for GripEncA {
        fn exec(&mut self) {
            self.shared().grip_quad.lock(|q| grip_edge(q, 3));
        }
    }

    #[task(binds = EXTI4, priority = 4, shared = [grip_quad])]
    struct GripEncB;
    impl RticTask for GripEncB {
        fn exec(&mut self) {
            self.shared().grip_quad.lock(|q| grip_edge(q, 4));
        }
    }

    #[task(binds = EXTI15_10, priority = 4, shared = [sonar_mm])]
    struct SonarEcho {
        capture: EchoCapture,
    }
    impl RticTask for SonarEcho {
        fn exec(&mut self) {
            let now = DWT::cycle_count();
            let exti = unsafe { &*pac::EXTI::ptr() };
            let pending = exti.pr().read().bits();
            let idr = unsafe { (*pac::GPIOB::ptr()).idr().read().bits() };
            let mut done: [Option<u16>; 4] = [None; 4];
            for (ch, bit) in ECHO_BITS.iter().enumerate() {
                if pending & (1 << bit) != 0 {
                    exti.pr().write(|w| unsafe { w.bits(1 << bit) });
                    done[ch] = self.capture.edge(ch, idr & (1 << bit) != 0, now, CPU_HZ);
                }
            }
            if done.iter().any(Option::is_some) {
                self.shared().sonar_mm.lock(|mm| {
                    for (slot, d) in mm.iter_mut().zip(done) {
                        if let Some(d) = d {
                            *slot = d;
                        }
                    }
                });
            }
        }
    }

    #[async_task(priority = 2, shared = [usb, latest_cmd, grip_quad, sonar_mm])]
    struct Control {
        drive_pwm: DrivePwm,
        grip_pwm: GripPwm,
        qei_left: Qei<TIM2>,
        qei_right: Qei<TIM3>,
        adc: Adc<ADC1>,
        vbat: PA4<Analog>,
        led: PC13<Output<PushPull>>,
        iwdg: IndependentWatchdog,
        controller: Controller,
    }
    impl RticAsyncTask for Control {
        type SpawnInput = ();
        async fn exec(&mut self, _: ()) {
            let (mut last_l, mut last_r) = (self.qei_left.count(), self.qei_right.count());
            let (mut enc_l, mut enc_r) = (0i32, 0i32);
            let mut tick: u32 = 0;
            let mut seq: u8 = 0;
            let mut next = Mono::now();
            loop {
                next += 10.millis();
                Mono::delay_until(next).await;
                self.iwdg.feed();
                tick = tick.wrapping_add(1);

                let (l, r) = (self.qei_left.count(), self.qei_right.count());
                enc_l = enc_l.wrapping_add(l.wrapping_sub(last_l) as i16 as i32);
                enc_r = enc_r.wrapping_add(r.wrapping_sub(last_r) as i16 as i32);
                (last_l, last_r) = (l, r);

                let raw: u16 = self.adc.read(&mut self.vbat).unwrap_or(0);
                let cmd = self.shared().latest_cmd.lock(|c| c.take());
                let enc_grip = self.shared().grip_quad.lock(|q| q.count());
                let out = self.controller.step(&Inputs {
                    now_ms: Mono::now().duration_since_epoch().to_millis(),
                    cmd,
                    battery_raw_mv: adc_to_mv(raw),
                    enc_grip,
                });

                drive(&mut self.drive_pwm, Channel::C1, Channel::C2, out.left);
                drive(&mut self.drive_pwm, Channel::C3, Channel::C4, out.right);
                drive(&mut self.grip_pwm, Channel::C1, Channel::C2, out.grip);

                if tick % 5 == 0 {
                    seq = seq.wrapping_add(1);
                    let sonar_mm = self.shared().sonar_mm.lock(|m| *m);
                    let t = Telemetry {
                        seq,
                        battery_mv: out.battery_mv,
                        enc_left: enc_l,
                        enc_right: enc_r,
                        enc_grip,
                        grip_state: out.grip_state,
                        box_count: out.box_count,
                        sonar_mm,
                        estop_latched: out.estop_latched,
                        faults: out.faults,
                    };
                    let mut frame = [0u8; MAX_FRAME];
                    let n = t.encode(&mut frame);
                    self.shared().usb.lock(|usb| {
                        let _ = usb.serial.write(&frame[..n]);
                    });
                }
                if tick % 50 == 0 {
                    self.led.toggle();
                }
            }
        }
    }

    #[async_task(priority = 1)]
    struct SonarTrigger {
        front: PB0<Output<PushPull>>,
        rear: PB1<Output<PushPull>>,
    }
    impl RticAsyncTask for SonarTrigger {
        type SpawnInput = ();
        async fn exec(&mut self, _: ()) {
            let mut front = true;
            loop {
                // 10us trigger pulse; left and right of a pair fire together
                if front {
                    self.front.set_high();
                    cortex_m::asm::delay(CPU_HZ / 100_000);
                    self.front.set_low();
                } else {
                    self.rear.set_high();
                    cortex_m::asm::delay(CPU_HZ / 100_000);
                    self.rear.set_low();
                }
                front = !front;
                Mono::delay(33.millis()).await;
            }
        }
    }
}
