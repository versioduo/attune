// Voltek Labs - attune
//
// © Kay Sievers 2020
// Unpublished work. Do not distribute.

#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2Power.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("net.voltek-labs.attune", 1, "versioduo:samd:step");

static V2LED LED(4, PIN_LED_WS2812, &sercom2, SERCOM2, SERCOM2_DMAC_ID_TX, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2MIDI::USBDevice MIDIDevice;
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);

enum { DRIVER_RAIL = 1, DRIVER_SOLENOID, DRIVER_LAMP };

static class Lamp : public V2Stepper::Power {
public:
  Lamp(const Power::Config conf, uint8_t index) :
    Power(conf, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void trigger(uint8_t channel, float power, float seconds) {
    if (seconds < 0.01f) {
      _pulse[channel].duration == 0;
      scaleVoltage(channel, 0);
      return;
    }

    scaleVoltage(channel, power);
    _pulse[channel].usec     = micros();
    _pulse[channel].duration = seconds * 1000000;
    scaleVoltage(channel, power);
  }

  void loop() {
    for (uint8_t i = 0; i < 2; i++) {
      if (_pulse[i].duration == 0)
        continue;

      if ((unsigned long)(micros() - _pulse[i].usec) < _pulse[i].duration)
        continue;

      _pulse[i].duration = 0;
      scaleVoltage(i, 0);
    }
  }

private:
  const uint8_t _index;
  struct {
    unsigned long usec;
    unsigned long duration;
  } _pulse[2] = {};

  void handleScaleVoltage(uint8_t channel, float fraction) {
    LED.setHSV(_index, V2LED::Cyan, 1, fraction);
  }
} Lamp({.ampere = 1}, DRIVER_LAMP);

static class Solenoid : public V2Stepper::Power {
public:
  Solenoid(const Power::Config conf, uint8_t index) :
    Power(conf, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void trigger(uint8_t channel, float power, float seconds) {
    if (seconds < 0.01f) {
      _pulse[channel].duration = 0;
      scaleVoltage(channel, 0);
      return;
    }

    scaleVoltage(channel, power);
    _pulse[channel].usec     = micros();
    _pulse[channel].duration = seconds * 1000000;
    scaleVoltage(channel, power);
  }

  void loop() {
    for (uint8_t i = 0; i < 2; i++) {
      if (_pulse[i].duration == 0)
        continue;

      if ((unsigned long)(micros() - _pulse[i].usec) < _pulse[i].duration)
        continue;

      _pulse[i].duration = 0;
      scaleVoltage(i, 0);
    }
  }

private:
  const uint8_t _index;
  struct {
    unsigned long usec;
    unsigned long duration;
  } _pulse[2] = {};

  void handleScaleVoltage(uint8_t channel, float fraction) {
    LED.setHSV(_index, V2LED::Magenta, 1, fraction);
  }
} Pulse({.ampere = 1.5}, DRIVER_SOLENOID);

static class Stepper : public V2Stepper::Motor {
public:
  Stepper(const Motor::Config conf, uint8_t index) :
    Motor(conf, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

  void positionNote(uint8_t note, float speed) {
    // Do not move the head while the trigger is in-between the keys.
    if ((unsigned long)(micros() - _usec) < 250000)
      return;

    position(getNotePosition(note), speed);
  }

  void playNote(uint8_t note, float volume) {
    if (getNotePosition(note) != getPosition())
      return;

    const float v = 0.5 + (0.5 * volume);
    Pulse.trigger(V2Music::Keyboard::isBlackKey(note) ? 0 : 1, v, 0.15);
    _usec = micros();

    Lamp.trigger(0, 0.6, 0.3);
  }

private:
  const uint8_t _index;
  unsigned long _usec = 0;

  uint32_t getNotePosition(uint8_t note) {
    return getNoteSteps(note, 0.166);
  }

  uint32_t getNoteSteps(uint8_t note, float octave_length) {
    // 40 teeth wheel == 80 mm.
    const float turns = V2Music::Keyboard::getKeyDistance(note, octave_length) / 0.08;
    return turns * 200;
  }

  void handleMovement(Move move) override {
    switch (move) {
      case Move::Forward:
        Lamp.trigger(0, 0.2, 3);
        LED.setHSV(_index, V2LED::Blue, 1, 0.5);
        break;

      case Move::Reverse:
        Lamp.trigger(0, 0.2, 3);
        LED.setHSV(_index, V2LED::Yellow, 1, 0.5);
        break;

      case Move::Stop:
        // Lamp.trigger(0, 0, 0);
        LED.setBrightness(_index, 0);
        break;
    }
  }
} Stepper({.ampere           = 0.6,
           .microsteps_shift = 2,
           .multisteps_shift = 2,
           .home             = {.speed = 200, .stall = 0.07},
           .speed            = {.min = 25, .max = 2000, .accel = 2000}
          },
          DRIVER_RAIL);

static class Power : public V2Power {
public:
  Power() : V2Power( {
    .min = 6, .max = 26
  }, PIN_VOLTAGE_SENSE) {}

  void begin() {
    pinMode(PIN_DRIVER_ENABLE, OUTPUT);
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

private:
  void handleOn() override {
    digitalWrite(PIN_DRIVER_ENABLE, LOW);
  }

  void handleOff() override {
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

  void handleNotify(float voltage) override {
    // Power-loss or commands without a power connection show yellow LEDs.
    if (voltage < config.min) {
      LED.splashHSV(0.5, V2LED::Yellow, 1, 0.5);
      return;
    }

    // Over-voltage shows red LEDs.
    if (voltage > config.max) {
      LED.splashHSV(0.5, V2LED::Red, 1, 1);
      return;
    }

    // The number of green LEDs shows the voltage.
    float fraction = voltage / (float)config.max;
    uint8_t n      = ceil((float)LED.getNumLEDs() * fraction);
    LED.splashHSV(0.5, n, V2LED::Green, 1, 0.5);
  }
} Power;

static class Device : public V2Device {
public:
  Device() : V2Device() {
    metadata.vendor      = "Voltek Labs";
    metadata.product     = "attune";
    metadata.description = "Piano One";
    metadata.home        = "https://voltek-labs.net/attune";

    system.download = "https://versioduo.com/download";

    configuration = {.magic = 0x9e020000 | USB_PID, .size = sizeof(config), .data = &config};
  }

  // 88 notes. The middle C, MIDI note 60, in this mapping is C3.
  static constexpr struct {
    uint8_t start;
    uint8_t count;
  } notes = {.start = V2MIDI::A(-1), .count = 85};

  // Config, written to EEPROM.
  struct {
    uint8_t channel;
  } config = {
    .channel = 0,
  };

  void reset() {
    digitalWrite(PIN_LED_ONBOARD, LOW);
    LED.reset();
    Lamp.reset();
    Pulse.reset();
    Stepper.reset();
    _volume    = 1;
    _home      = false;
    _speed_max = 1;
    Power.off();
  }

  void allNotesOff() {
    if (!power())
      return;

    // ~1.3m maximum, 40 teeth wheel == 80 mm.
    Stepper.home((1.3f / 0.08f) * 200.f, 158);
    _home = true;
  }

  void loop() {}

private:
  float _volume    = 1;
  bool _home       = false;
  float _speed_max = 1;

  bool power() {
    bool continuous;
    if (!Power.on(continuous))
      return false;

    if (!continuous) {
      Lamp.reset();
      Pulse.reset();
      Stepper.reset();
      _home = false;
    }

    return true;
  }

  void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) override {
    if (note < notes.start)
      return;

    if (note >= notes.start + notes.count)
      return;

    if (!power())
      return;

    if (!_home)
      allNotesOff();

    if (velocity == 0) {
      Stepper.playNote(note, _volume);
      return;
    }

    const float fraction = (float)velocity / 127;
    Stepper.positionNote(note, fraction * _speed_max);
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    switch (controller) {
      case V2MIDI::ModulationWheel:
        _speed_max = (float)value / 127;
        break;

      case V2MIDI::BreathController: {
        const float fraction = (float)value / 127;
        Lamp.trigger(0, 0.6f * fraction, 10);
        break;
      }

      case V2MIDI::ChannelVolume:
        _volume = (float)value / 127;
        break;

      case V2MIDI::AllNotesOff:
        allNotesOff();

        break;
    }
  }

  void handleSystemReset() override {
    reset();
  }

  void exportInput(JsonObject json) override {
    JsonArray json_controller = json.createNestedArray("controllers");
    JsonObject json_speed     = json_controller.createNestedObject();
    json_speed["name"]        = "Speed";
    json_speed["number"]      = (uint8_t)V2MIDI::ModulationWheel;
    json_speed["value"]       = _speed_max * 127;

    JsonObject json_light     = json_controller.createNestedObject();
    json_light["name"]        = "Light";
    json_light["number"]      = (uint8_t)V2MIDI::BreathController;
    json_light["value"]       = 0;

    JsonObject json_volume = json_controller.createNestedObject();
    json_volume["name"]    = "Volume";
    json_volume["number"]  = (uint8_t)V2MIDI::ChannelVolume;
    json_volume["value"]   = _volume * 127;

    JsonObject json_chromatic = json.createNestedObject("chromatic");
    json_chromatic["start"]   = notes.start;
    json_chromatic["count"]   = notes.count;
  }

  void exportOutput(JsonObject json) override {
    json["channel"] = config.channel;
  }

  void importConfiguration(JsonObject json) override {
    if (!json["channel"].isNull()) {
      uint8_t channel = json["channel"];

      if (channel < 1)
        config.channel = 0;

      else if (channel > 16)
        config.channel = 15;

      else
        config.channel = channel - 1;
    }
  }

  void exportConfiguration(JsonObject json) override {
    json["#channel"] = "The MIDI channel to send control values and notes";
    json["channel"]  = config.channel + 1;
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
public:
  MIDI() {
    _midi.setSystemExclusiveBuffer(_sysex_buffer, sizeof(_sysex_buffer));
  }

  void loop() {
    if (!MIDIDevice.receive(&_midi))
      return;

    if (_midi.getPort() == 0) {
      // Simple messages are dispatched immediately, if it is sysex message,
      // we store the chunk of the message in our packet and receive() again
      // until it is complete.
      if (_midi.storeSystemExclusive())
        Device.dispatchMIDI(&MIDIDevice, &_midi);

    } else {
      _midi.setPort(_midi.getPort() - 1);
      Socket.send(&_midi);
    }
  }

private:
  V2MIDI::Packet _midi;
  uint8_t _sysex_buffer[12288];
} MIDI;

// Dispatch Link packets
static class Link : public V2Link {
public:
  Link() : V2Link(&Plug, &Socket) {
    _midi.setSystemExclusiveBuffer(_sysex_buffer, sizeof(_sysex_buffer));
  }

private:
  // Receive a host event from our parent device
  void receivePlug(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      packet->receive(&_midi);
      if (_midi.storeSystemExclusive())
        Device.dispatchMIDI(&Plug, &_midi);
    }
  }

  // Forward children device events to the host
  void receiveSocket(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      uint8_t address = packet->getAddress();
      if (address == 0x0f)
        return;

      if (MIDIDevice.connected()) {
        packet->receive(&_midi);
        _midi.setPort(address + 1);
        MIDIDevice.send(&_midi);
      }
    }
  }

private:
  V2MIDI::Packet _midi;
  uint8_t _sysex_buffer[12288];
} Link;

void TC3_Handler() {
  noInterrupts();

  Stepper.tick();

  TC3->COUNT16.INTFLAG.bit.MC0 = 1;
  interrupts();
}

class Timer {
public:
  static void begin() {
    // Connect clock generator 1, 48MHz.
    GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    while (GCLK->SYNCBUSY.reg > 0)
      ;

    TC3->COUNT16.CTRLA.bit.ENABLE = 0;

    // Enable match frequency mode.
    TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    while (TC3->COUNT16.SYNCBUSY.reg != 0)
      ;

    // Enable the compare interrupt.
    TC3->COUNT16.INTENSET.reg     = 0;
    TC3->COUNT16.INTENSET.bit.MC0 = 1;

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1;
    while (TC3->COUNT16.SYNCBUSY.reg != 0)
      ;

    // 48MHz clock / 200kHz timer = 240 ticks / period.
    TC3->COUNT16.CC[0].reg = 240;
    while (TC3->COUNT16.SYNCBUSY.reg != 0)
      ;

    // Enable IRQ.
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.SYNCBUSY.reg != 0)
      ;
  }
};

void setup() {
  Serial.begin(9600);
  SPI.begin();

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_DRIVER_SELECT + i, OUTPUT);
    digitalWrite(PIN_DRIVER_SELECT + i, HIGH);

    pinMode(PIN_DRIVER_STEP + i, OUTPUT);
    digitalWrite(PIN_DRIVER_STEP + i, LOW);
  }

  Device.begin();

  static Adafruit_USBD_WebUSB WebUSB;
  static WEBUSB_URL_DEF(WEBUSBLandingPage, 1 /*https*/, "versioduo.com/configure");
  WebUSB.begin();
  WebUSB.setLandingPage(&WEBUSBLandingPage);

  LED.begin();
  LED.setMaxBrightness(0.5);

  uint8_t ports = 1;
  if (Device.system.ports.reboot > 1)
    ports = Device.system.ports.reboot;

  else if (Device.system.ports.configured > 1)
    ports = Device.system.ports.configured;

  if (ports > 1) {
    Device.system.ports.current = ports;
    MIDIDevice.setPorts(ports);

    // Operating systems/services/apps get confused if the number
    // of ports changes between device connections; some hang, some
    // don't probe the device again and ignore the new number of ports.
    //
    // To work around it, let the USB ID depend on the number of ports.
    USBDevice.setID(USB_VID, USB_PID + ports - 1);
  }

  MIDIDevice.begin();
  Plug.begin();
  Socket.begin();

  Power.begin();
  Lamp.begin();
  Pulse.begin();
  Stepper.begin();

  Timer::begin();
  Device.reset();
}

void loop() {
  LED.loop();
  MIDI.loop();
  Link.loop();
  Device.loop();
  Power.loop();

  Stepper.loop();
  Lamp.loop();
  Pulse.loop();
}
