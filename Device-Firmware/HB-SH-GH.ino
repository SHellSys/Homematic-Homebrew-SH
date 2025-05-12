//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
//- -----------------------------------------------------------------------------------------------------------------------
// #define NDEBUG   // disable all serial debuf messages
// #define NSENSORS // if defined, only fake values are used
#define SENSOR_ONLY

#define  EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <SPI.h>
#include <AskSinPP.h>
#include <Register.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <HTU21D.h>

#include <MultiChannelDevice.h>

#define CONFIG_BUTTON_PIN                    8     // Anlerntaster-Pin

#define dryAnalogValue 1023
#define wetAnalogValue 0
#define dryValuePercent 0
#define wetValuePercent 100

#define PEERS_PER_CHANNEL          4

using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xF1, 0xD0, 0x02},        // Device ID
  "HBSHGH0001",           	 // Device Serial
  {0xF1, 0xD0},            	 // Device Model
  0x14,                   	 // Firmware Version
  as::DeviceType::THSensor,  // Device Type
  {0x01, 0x01}             	 // Info Bytes
};

// Configure the used hardware
typedef AskSin<NoLed, NoBattery, Radio<LibSPI<10>, 2>> Hal;
Hal hal;

class WeatherEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int16_t temp, uint16_t airPressure, uint8_t humidity, uint32_t brightness, uint8_t moisture1, uint8_t moisture2, uint8_t moisture3, uint8_t moisture4) {
      Message::init(0x1a, msgcnt, 0x70, BIDI | RPTEN, (temp >> 8) & 0x7f, temp & 0xff);
      pload[0] = (airPressure >> 8) & 0xff;
      pload[1] = airPressure & 0xff;
      pload[2] = humidity;
      pload[3] = (brightness >>  16) & 0xff;
      pload[4] = (brightness >>  8) & 0xff;
      pload[5] = brightness & 0xff;
      pload[6] = moisture1;
      pload[7] = moisture2;
      pload[8] = moisture3;
      pload[9] = moisture4;
    }
};

DEFREGISTER(Reg0, MASTERID_REGS, DREG_TRANSMITTRYMAX, 0x20, 0x21, 0x22, 0x23)
class SensorList0 : public RegList0<Reg0> {
  public:
    SensorList0(uint16_t addr) : RegList0<Reg0>(addr) {}

    bool updIntervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t updIntervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      updIntervall(60);
      transmitDevTryMax(6);
    }
};

class WeatherChannel : public Channel<Hal, EmptyList, EmptyList, List4, PEERS_PER_CHANNEL, SensorList0>, public Alarm {
    int16_t       temperature;
    uint16_t      airPressure;
    uint8_t       humidity;
    uint32_t      brightness;
    uint8_t       moisture1;
    uint8_t       moisture2;
    uint8_t       moisture3;
    uint8_t       moisture4;

    bool          initComplete;

    Adafruit_BMP085 bmpSen;
    BH1750 lightSen;
    HTU21D humiditySen;

  public:
    WeatherChannel () : Channel(), Alarm(seconds2ticks(60)), temperature(0), airPressure(0), humidity(0), brightness(0), moisture1(0), moisture2(0), moisture3(0), moisture4(0)  {}
    virtual ~WeatherChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      measure_thpb();

      WeatherEventMsg& msg = (WeatherEventMsg&)device().message();
      uint8_t msgcnt = device().nextcount();
      msg.init(msgcnt, temperature, airPressure, humidity, brightness, moisture1, moisture2, moisture3, moisture4);
      if (msgcnt % 20 == 1) {
        device().sendMasterEvent(msg);
      } else {
        device().broadcastEvent(msg);
      }
      uint16_t updCycle = this->device().getList0().updIntervall();
      tick = seconds2ticks(updCycle);

      initComplete = true;
      sysclock.add(*this);
    }

    void measure_thpb() {
      temperature = bmpSen.readTemperature();
      airPressure = bmpSen.readPressure();
      humidity    = humiditySen.getHumidity();
      brightness = lightSen.readLightLevel();
      moisture1 = map(analogRead(A0), dryAnalogValue, wetAnalogValue, dryValuePercent, wetValuePercent);
      moisture2 = map(analogRead(A1), dryAnalogValue, wetAnalogValue, dryValuePercent, wetValuePercent);
      moisture3 = map(analogRead(A2), dryAnalogValue, wetAnalogValue, dryValuePercent, wetValuePercent);
      moisture4 = map(analogRead(A3), dryAnalogValue, wetAnalogValue, dryValuePercent, wetValuePercent);
    }

    void setup(Device<Hal, SensorList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      tick = seconds2ticks(3);	// first message in 3 sec.

      Wire.begin();

      pinMode(A0, INPUT);
      pinMode(A1, INPUT);
      pinMode(A2, INPUT);
      pinMode(A3, INPUT);

      bmpSen.begin();
      lightSen.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      humiditySen.begin();

      sysclock.add(*this);
    }

    void configChanged() {
      DPRINTLN("* Config changed       : List1");
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class SensChannelDevice : public MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> {
  public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> TSDevice;
    SensChannelDevice(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
    virtual ~SensChannelDevice () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINTLN("* Config Changed       : List0");
      DPRINT(F("* SENDEINTERVALL       : ")); DDECLN(this->getList0().updIntervall());
      //DPRINT(F("* ALTITUDE             : ")); DDECLN(this->getList0().height());
      //DPRINT(F("* TRANSMITTRYMAX       : ")); DDECLN(this->getList0().transmitDevTryMax());
    }
};

SensChannelDevice sdev(devinfo, 0x20);
ConfigButton<SensChannelDevice> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
  //sdev.startPairing();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    // hal.activity.savePower<Idle<false, true>>(hal);
  }
}
