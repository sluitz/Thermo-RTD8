// Firmware for 8-channel MAX31865 RTD board

// Requires the following libraries (all available as downloads in the Arduino Library Manager):
// * Adafruit_MAX31865
// * QNEthernet
// * ArduinoRS485
// * ArduinoModbus
// * Watchdog (by Peter Polidoro)


// Board Notes:
//
// 2/3/4-wire needs to be set both by jumper configuration and the readout software via modbus (saved in EEPROM)
// Two rows of 4 chips and connectors. Jumpers and connector pinouts are rotated by 180 degrees in bottom row.
//
//
// Top row jumpers (channels 1-4)
//        | L(2 pin) | R(2 pin) | 3 (3-pin) |
// 2-wire | closed   | closed   | left      |
// 3-wire | open     | closed   | right     |
// 4-wire | open     | open     | left      |

// Bottom row jumpers (channels 5-8)
//        | L(2 pin) | R(2 pin) | 3 (3-pin) |
// 2-wire | closed   | closed   | right     |
// 3-wire | closed   | open     | left      |
// 4-wire | open     | open     | right     |
//
// Note that the jumper table printed on the v7/2023 board does not correctly show the 3-pin jumper settings

// Modbus registers

// HoldingRegister H[ChannelNumber]: mode (0=2/4-wire, 1=3-wire)
// InputRegister I[2*ChannelNumber],I[2*ChannelNumber+1]: 32-bit float temperature [degC]

#include <Adafruit_MAX31865.h>
#include <QNEthernet.h>
#include <ArduinoRS485.h>   // Required by ArduinoModbus
#include <ArduinoModbus.h>  //
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Watchdog.h>



#define DEBUG


// Network settings
using namespace qindesign::network;  // needed for QNEthernet

// static IP address configuration. We will try DHCP first and assign the static IP if DHCP times out
IPAddress staticIP(192, 168, 1, 77);
IPAddress gateway(192, 168, 1, 254);
IPAddress subnetMask(255, 255, 255, 0);
IPAddress nameserver(192, 168, 1, 254);

const uint32_t kDHCPTimeout = 15000;  // DHCP timeout, in milliseconds.
const uint32_t kLinkTimeout = 5000;   // Link timeout, in milliseconds.

// RTD/board settings

const float R_ref = 430.0;      // reference resistor (same for all channels)
const float R_nominal = 100.0;  // nominal resistance of RTD (default)
const uint8_t NCHIPS = 8;       // actual number of channels/chips (<= NCHIPS)

// Modbus
const uint16_t modbus_port = 502;

// other settings
const uint8_t EEPROM_BASE_CONF = 40;
const uint8_t EEPROM_CKSUM_INIT = 0xcc;
const uint8_t rtd_cs[NCHIPS] = { 35, 41, 18, 34, 33, 26, 29, 38 };  // CS pins of the MAX chips


// default mode for chips (2/4-wire)
uint8_t defmode[NCHIPS] = { MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE, MAX31865_2WIRE };
uint8_t mode[NCHIPS];

Adafruit_MAX31865* rtd[NCHIPS];
uint16_t rawRTD[NCHIPS];
float rtdtemp[NCHIPS];

const unsigned int LOOPDELAY = 100;

// Modbus Server on port 502
EthernetServer ethServer(modbus_port);
ModbusTCPServer modbusTCPServer;

Watchdog watchdog;

void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 4000) {};

  initEthernet();

  printf("Starting...\r\n");

  ethServer.begin();
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1)
      ;
  }

  // set up modbus registers
  modbusTCPServer.configureHoldingRegisters(0, NCHIPS);
  modbusTCPServer.configureInputRegisters(0, 2 * NCHIPS + NCHIPS);

  // initalize mode registers from EEPROM if possible
  if (EEPROM_read_conf() == 0) {
    // use default if we can't read from EEPROM
    Serial.println("Initializing EEPROM with default settings");
    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      modbusTCPServer.holdingRegisterWrite(chip, defmode[chip]);
    }
  }

  Serial.println("Initializing and starting MAX31865 chips");
  // instantiate and initialize RTD chips
  Serial.print("Chip mode: ");
  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    rtd[chip] = new Adafruit_MAX31865(rtd_cs[chip]);
    delay(10);
    mode[chip] = modbusTCPServer.holdingRegisterRead(chip);
    printf("%d:%d, ", chip, mode[chip]);
    rtd[chip]->begin((max31865_numwires_t)mode[chip]);
  }
  printf("\n");
  watchdog.enable(Watchdog::TIMEOUT_4S);
}

void loop() {

  uint16_t inputregs[2 * NCHIPS];

  // read RTDs and update modbus registers
  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    rawRTD[chip] = rtd[chip]->readRTD();
    rtdtemp[chip] = 0.0;
    if (!rtd[chip]->readFault()) rtdtemp[chip] = rtd[chip]->temperature(R_nominal, R_ref) + 273.15;  // we output K
    enc_float(&inputregs[2 * chip], rtdtemp[chip]);
    #ifdef DEBUG
    Serial.print(chip); Serial.print(": "); Serial.print(rtdtemp[chip]); Serial.print(" ");
    #endif
    serveNetwork();
  }
  
  Serial.println();

  for (uint8_t chip = 0; chip < NCHIPS; chip++) {
    uint16_t newmode = modbusTCPServer.holdingRegisterRead(chip);
    if (newmode != mode[chip]) {
      Serial.print("chip "); Serial.print(chip); Serial.print(" new mode:"); Serial.println(newmode);
      //printf("chip %d, mode changed to %d\r\n", chip, newmode);
      rtd[chip]->begin((max31865_numwires_t)newmode);
      mode[chip] = newmode;
    }
  }

  modbusTCPServer.writeInputRegisters(0, inputregs, 2 * NCHIPS);
  EEPROM_write_conf();
  
  watchdog.reset();
  delay(100);
}



void serveNetwork() {

  elapsedMillis sinceDelayStart = 0;
  while (sinceDelayStart < LOOPDELAY) {
    EthernetClient client = ethServer.available();
    if (client) {
#ifdef DEBUG0
      Serial.println("new client");
      Serial.println();
#endif
      modbusTCPServer.accept(client);
      if (client.connected()) {
        modbusTCPServer.poll();
      }
    }
  }
}



void enc_float(uint16_t* t, float x) {

  // encode float in 2 uint16 for modbus

  union U {
    float f;
    uint32_t u;
  };
  union U u;
  u.f = x;
  t[0] = (u.u >> 16) & 0xffffU;
  t[1] = (u.u & 0xffffU);
}


void EEPROM_cond_write(int addr, byte val)
// only write values that are different

{
  if ((EEPROM.read((addr)) != (val))) {
    EEPROM.write(addr, val);
    #ifdef DEBUG0
    Serial.print("WRITE_COND"); Serial.print(addr); Serial.print(":"); Serial.println(val);
    #endif
  }
}

void EEPROM_write_conf() {

  int addr = EEPROM_BASE_CONF;
  uint8_t cksum = EEPROM_CKSUM_INIT;
  uint8_t val;

#ifdef DEBUG0
  Serial.println("Updating EEPROM configuration");
#endif
  for (unsigned int chip = 0; chip < NCHIPS; chip++) {
    val = modbusTCPServer.holdingRegisterRead(chip) & 0xff;
    EEPROM_cond_write(addr + chip, val);
    cksum ^= val;
  }
  // write checksum
  EEPROM_cond_write(addr + NCHIPS, cksum);
}

int EEPROM_read_conf() {

  uint8_t cksum = EEPROM_CKSUM_INIT;
  int addr = EEPROM_BASE_CONF;

#ifdef DEBUG0
  Serial.println("Reading EEPROM configuration");
#endif

  // verify checksum
  for (unsigned int chip = 0; chip < NCHIPS; chip++)
    cksum ^= EEPROM.read(addr + chip);

  if (cksum == EEPROM.read(addr + NCHIPS)) {
// checksum OK, read values
#ifdef DEBUG
    Serial.println("EEPROM: Checksum OK");
#endif
    for (unsigned int chip = 0; chip < NCHIPS; chip++) {
      uint16_t mode = EEPROM.read(addr + chip);
#ifdef DEBUG
      Serial.print("EEPROM read chip, mode:");
      Serial.print(chip);
      Serial.print(", ");
      Serial.println(mode);
#endif
      modbusTCPServer.holdingRegisterWrite(chip, mode);
    }
    return 1;
  } else {
// checksum not OK
#ifdef DEBUG0
    Serial.println("EEPROM: Checksum NOT OK");
#endif
    return 0;
  }
}

bool initEthernet() {

  byte mac[6];

  // get and print MAC address
  Ethernet.macAddress(mac);
  Serial.println();
  Serial.print("MAC address: ");
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Initialize Ethernet

  // Set up link change listener
  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  // Set up address change listener
  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    bool hasIP = (ip != INADDR_NONE);
    if (hasIP) {
      printf("[Ethernet] Address changed:\r\n");
      printf("    Local IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.subnetMask();
      printf("    Subnet   = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.gatewayIP();
      printf("    Gateway  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.dnsServerIP();
      if (ip != INADDR_NONE) {  // May happen with static IP
        printf("    DNS      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      }
    } else {
      printf("[Ethernet] Address changed: No IP address\r\n");
    }
  });


  printf("Attempting DHCP first\r\n");
  if (!Ethernet.begin()) {
    printf("Failed to start Ethernet\r\n");
    return false;
  }

  // We can choose not to wait and rely on the listener to tell us
  // when an address has been assigned
  if (kDHCPTimeout > 0) {
    printf("Waiting for IP address...\r\n");
    if (!Ethernet.waitForLocalIP(kDHCPTimeout)) {
      printf("DHCP timed out. Falling back to static IP\r\n");
      if (!Ethernet.begin(staticIP, subnetMask, gateway)) {
        printf("Failed to start Ethernet\r\n");
        return false;
      }
    }
  }

  // When setting a static IP, the address is changed immediately,
  // but the link may not be up; optionally wait for the link here
  if (kLinkTimeout > 0) {
    printf("Waiting for link...\r\n");
    if (!Ethernet.waitForLink(kLinkTimeout)) {
      printf("No link yet\r\n");
      // We may still see a link later, after the timeout, so
      // continue instead of returning
    }
  }
  return true;
}
