#include "dot_util.h"
#include "RadioEvent.h"

#if ACTIVE_EXAMPLE == AUTO_OTA_EXAMPLE

/////////////////////////////////////////////////////////////////////////////
// -------------------- DOT LIBRARY REQUIRED ------------------------------//
// * Because these example programs can be used for both mDot and xDot     //
//     devices, the LoRa stack is not included. The libmDot library should //
//     be imported if building for mDot devices. The libxDot library       //
//     should be imported if building for xDot devices.                    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-mbed5/        //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-mbed5/        //
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
// * these options must match the settings on your gateway //
// * edit their values to match your configuration         //
// * frequency sub band is only relevant for the 915 bands //
// * either the network name and passphrase can be used or //
//     the network ID (8 bytes) and KEY (16 bytes)         //
/////////////////////////////////////////////////////////////
static uint8_t network_id[] = { 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18 };
static uint8_t network_key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16 };
static uint8_t frequency_sub_band = 0;
static bool public_network = true;
static uint8_t ack = 0;
uint8_t tx_datarate = lora::DR_5;
uint8_t tx_power = 14;
static bool adr = false;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

// RS485
Serial RS485(PA_2, PA_3);        // D1, D0
DigitalOut tx_enable1(PA_1);     // D6
DigitalOut tx_enable2(PA_11);    // D7
DigitalOut tx_enable_led(LED1);  // D3
uint8_t regData[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xcb};
uint8_t regValue[9];

Serial pc(USBTX, USBRX);

// sensor data
float solar;
float temp;

void push_sensor_data(vector<uint8_t> &tx_data, uint8_t code, float value) {
    int buf;
    memcpy(&buf, &value, sizeof(value));
    
    tx_data.push_back(code);
    for (int i=3; i>=0; i--) {
        tx_data.push_back(buf >> i*8 & 0xff);
    }
}

void print_data(string prefix, vector<uint8_t> &tx_data) {
    string hexString = "";
    
    for(int i = 0; i < tx_data.size(); i++) {
        uint8_t j;
        j = tx_data[i] >> 4;
        hexString += j < 10 ? j + 48 : j + 55; 
        j = tx_data[i] & 0x0f;
        hexString += j < 10 ? j + 48 : j + 55; 
    }
    
    logInfo("%s: %s", prefix.c_str(), hexString.c_str());
}

void print_data(string prefix, uint8_t *tx_data, int size) {
    string hexString = "";
    
    for(int i = 0; i < size; i++) {
        uint8_t j;
        j = tx_data[i] >> 4;
        hexString += j < 10 ? j + 48 : j + 55; 
        j = tx_data[i] & 0x0f;
        hexString += j < 10 ? j + 48 : j + 55; 
    }
    
    logInfo("%s: %s", prefix.c_str(), hexString.c_str());
}

void getData() {
    tx_enable_led = 1;
    tx_enable1 = 1;
    tx_enable2 = 1;
    for (int i = 0; i < 8; i++) {
        RS485.putc(regData[i]);
    }
    print_data("write", regData, 8);

    tx_enable_led = 0;
    tx_enable1 = 0;
    tx_enable2 = 0;
    wait_ms(1000);
    if(RS485.readable() >0){
    memset(regValue, 0, sizeof(regValue));    
    for (int i = 0; i < 9; i++) {
        regValue[i] = RS485.getc();
    }
    }else logInfo("cant receive");
    print_data("read", regValue, 9);
       
    uint16_t a = 0;
    a = 0;
    a |= regValue[3];
    a = (a << 8) | regValue[4];
    solar = (float)a;
    
    int16_t b = 0;
    b = 0;
    b |= regValue[5];
    b = (b << 8) | regValue[6];
    temp = (float)b / 10;
}

int main() {
    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    // RS485
    RS485.baud(9600);
    pc.baud(115200);
    
#if CHANNEL_PLAN == CP_US915
    plan = new lora::ChannelPlan_US915();
#elif CHANNEL_PLAN == CP_AU915
    plan = new lora::ChannelPlan_AU915();
#elif CHANNEL_PLAN == CP_EU868
    plan = new lora::ChannelPlan_EU868();
#elif CHANNEL_PLAN == CP_KR920
    plan = new lora::ChannelPlan_KR920();
#elif CHANNEL_PLAN == CP_AS923
    plan = new lora::ChannelPlan_AS923();
#elif CHANNEL_PLAN == CP_AS923_JAPAN
    plan = new lora::ChannelPlan_AS923_Japan();
#elif CHANNEL_PLAN == CP_IN865
    plan = new lora::ChannelPlan_IN865();
#endif
    assert(plan);

    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    if (!dot->getStandbyFlag()) {
        logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

        // update configuration if necessary
        if (dot->getJoinMode() != mDot::OTA) {
            logInfo("changing network join mode to OTA");
            if (dot->setJoinMode(mDot::OTA) != mDot::MDOT_OK) {
                logError("failed to set network join mode to OTA");
            }
        }
        // in OTA and AUTO_OTA join modes, the credentials can be passed to the library as a name and passphrase or an ID and KEY
        // only one method or the other should be used!
        // network ID = crc64(network name)
        // network KEY = cmac(network passphrase)
        // update_ota_config_name_phrase(network_name, network_passphrase, frequency_sub_band, public_network, ack);
        update_ota_config_id_key(network_id, network_key, frequency_sub_band, public_network, ack, tx_datarate, tx_power);

        // configure network link checks
        // network link checks are a good alternative to requiring the gateway to ACK every packet and should allow a single gateway to handle more Dots
        // check the link every count packets
        // declare the Dot disconnected after threshold failed link checks
        // for count = 3 and threshold = 5, the Dot will ask for a link check response every 5 packets and will consider the connection lost if it fails to receive 3 responses in a row
         update_network_link_check_config(3, 5);

        // enable or disable Adaptive Data Rate
        dot->setAdr(adr);
    
        // save changes to configuration
        logInfo("saving configuration");
        if (!dot->saveConfig()) {
            logError("failed to save configuration");
        }

        // display configuration
        display_config();
    } else {
        // restore the saved session if the dot woke from deepsleep mode
        // useful to use with deepsleep because session info is otherwise lost when the dot enters deepsleep
        logInfo("restoring network session from NVM");
        dot->restoreNetworkSession();
    }

    while(1) {
        // join network if not joined
        if (!dot->getNetworkJoinStatus()) {
            join_network();
        }
        
        std::vector<uint8_t> tx_data;
        
        getData();
        logInfo("solar: %f", solar);
        logInfo("temp: %f", temp);
        
        push_sensor_data(tx_data, 3, solar);
        push_sensor_data(tx_data, 4, temp);
        
        print_data("tx_data", tx_data);
        send_data(tx_data);
    
        wait_ms(10000);
    }

 
    return 0;
}

#endif

