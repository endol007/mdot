#include "dot_util.h"
#include "RadioEvent.h"
#include "DHT22.h"

#if ACTIVE_EXAMPLE == OTA_EXAMPLE

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

DHT22 dht22(PA_4);
float temp, humi;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

Serial pc(USBTX, USBRX);

void print_tx_data(vector<uint8_t> &tx_data) {
    string hexString = "";
    
    for(int i = 0; i < tx_data.size(); i++) {
        uint8_t j;
        j = tx_data[i] >> 4;
        hexString += j < 10 ? j + 48 : j + 55; 
        j = tx_data[i] & 0x0f;
        hexString += j < 10 ? j + 48 : j + 55; 
    }
    
    logInfo("tx_data: %s", hexString.c_str());
}

void push_float(vector<uint8_t> &tx_data, uint8_t code, float value) {
    int buf;
    memcpy(&buf, &value, sizeof(value));
    
    for (int i=3; i>=0; i--) {
        tx_data.push_back(buf >> i*8 & 0xff);
    }
}

int main() {
    dht22.sample();
    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    pc.baud(115200);
    

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);
    
    
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
        // update_network_link_check_config(3, 5);

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

    while (true) {
    // 
        //     //join network if not joined
       if (!dot->getNetworkJoinStatus()) {
           join_network();
       }
        
        std::vector<uint8_t> tx_data;
        temp = dht22.getTemperature() / 10.0;
        humi = dht22.getHumidity() / 10.0;
        
         logInfo("Temperature: %.1f", temp);
         logInfo("Humidity: %.1f", humi);

        push_float(tx_data, 1, temp);
        push_float(tx_data, 2, humi);
        
        send_data(tx_data);
        print_tx_data(tx_data);
        
        wait_ms(5000);
    }
 
    return 0;
}

#endif
