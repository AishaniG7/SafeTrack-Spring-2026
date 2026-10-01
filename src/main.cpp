#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstdio>
#include "gpsPacket.h"
#include "meshPacket.h"
#include "node_identity.h"
#include "mesh_crypto.h"

//Hardware and Pins
#define LED (0 + 15)
#define OLED_RESET -1
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

int32_t counter = 0;
// Node and Mesh
//#define NODE_ID 1
//#define NODE_ID 2
#define NODE_ID 3

#define DUP_CACHE_SIZE 8 
#define MAX_NEIGHBORS 10
#define MAX_ROUTES 10
#define MESH_BROADCAST 0xFF


#define PRI_ACK   3
#define PRI_GPS   2
#define PRI_HELLO 1

//other definations
#define gpsSerial Serial1

//LoRa Modem Parametrers
const float FREQ_MHZ = 906.875;
const uint8_t SF = 11;
const unsigned long BW = 250000;
const uint8_t CR = 5;
const uint16_t PREAMBLE = 16;
const uint8_t SYNCWORD = 0x2B;

//Timing and State
volatile bool receivedFlag = false; // flag to indicate that a packet was received (IRQ)
int buttonState = 0;
int lastButtonState = 0; 
int showRX = 0; 

unsigned long lastRXActivity = 0;

volatile uint16_t pendingAckSeq = 0;
volatile bool ackReceivedFlag = false;
unsigned long ackWaitStart = 0;
bool pendingACK = false;
MeshPacket pendingPkt;
uint16_t pendingSeq = 0;
unsigned long pendingSentAt = 0;
uint8_t pendingRetries = 0;
#define MAX_RETRIES 3
#define ACK_TIMEOUT 5000

//Objects
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
TinyGPSPlus gps;

uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
char latBuffer[30]; // Buffer for latitude display
char lngBuffer[30]; // Buffer for longitude display
char dateBuffer[30]; // Buffer for date display
char recvBuffer[64]; // Buffer for LoRa recv display
char rssiBuffer[30]; // Buffer for rssi

double lastLat = 0;
double lastLng = 0;
TinyGPSDate lastDate;

//Data Structures
struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

struct Neighbor {
    uint8_t nodeID;
    int16_t rssi; 
    unsigned long lastSeen; // millis() timestamp
};

struct Route {
	uint8_t dest;
    uint8_t nextHop;
    uint8_t hops;
    int16_t linkRSSI;
    unsigned long lastUpdated; 
};

Route routes[MAX_ROUTES];

Neighbor neighbors[MAX_NEIGHBORS];

SeenPacket seenCache[DUP_CACHE_SIZE];
uint8_t seenIndex = 0;

// Declarations Functions
void setFlag(void);
void setup();
void loop();
String bytesToAscii(const uint8_t *, size_t);

//Role Definitions
//#define ROLE_TX
#define ROLE_RX

bool channelBusy() {
    return (millis() - lastRXActivity) < 600;
}

unsigned long getBackoff(uint8_t priority) {
    switch(priority) {
        case PRI_ACK:   return random(10, 50);
        case PRI_GPS:   return random(50, 150);
        case PRI_HELLO: return random(200, 600);
    }
    return 100;
}

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void readGPS() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    if (lat != lastLat || lng != lastLng) {
        lastLat = lat;
        lastLng = lng;

        display.setCursor(0, 0);
        display.print("Lat: ");
        display.print(lat, 6);

        display.setCursor(0, 10);
        display.print("Lng: ");
        display.print(lng, 6);

        display.display();
    }
}

bool seenBefore(uint8_t src, uint16_t seq) {
  // check cache
  for (int i = 0; i < DUP_CACHE_SIZE; i++) {
    if (seenCache[i].src == src &&
        seenCache[i].seq == seq) {
      return true;  // duplicate found
    }
  }

  // not seen then store it (circular buffer)
  seenCache[seenIndex].src = src;
  seenCache[seenIndex].seq = seq;
  seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;

  return false;
}

bool hasSeen(uint8_t src, uint16_t seq) {
    for (int i = 0; i < DUP_CACHE_SIZE; i++) {
        if (seenCache[i].src == src && 
            seenCache[i].seq == seq) {
            return true;
        }
    }
    return false;
}

void markSeen(uint8_t src, uint16_t seq) {
    seenCache[seenIndex].src = src;
    seenCache[seenIndex].seq = seq;
    seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;
}

void sendHello() {
    //const unsigned long HELLO_INTERVAL = 15000;
    static unsigned long lastHello = 0;

    const unsigned long HELLO_BASE = 20000;
    const unsigned long HELLO_JITTER = 5000;

    static unsigned long nextHelloIn = HELLO_BASE + random(0, HELLO_JITTER);

    if (millis() - lastHello > nextHelloIn) {
        if (channelBusy()) {
            Serial.println("Channel busy, backing off HELLO");

            delay(getBackoff(PRI_HELLO));
            return;
        }

        Serial.print(millis() / 1000);
        Serial.println("s: Sending HELLO");

        lastHello = millis();
        nextHelloIn = HELLO_BASE + random(0, HELLO_JITTER) + NodeIdentity::getNodeId() * 37;

        static uint16_t seqCounter = 0;

        MeshPacket pkt = {};
        pkt.src = NodeIdentity::getNodeId();
        pkt.lastHop = NodeIdentity::getNodeId();
        pkt.dst = MESH_BROADCAST;
        pkt.ttl = 1;
        pkt.seq = seqCounter++;
        pkt.type = PKT_HELLO;

        // Build route advertisement
        HelloPayload advert = {};
        for (int i = 0; i < MAX_ROUTES; i++) {
            if (routes[i].dest != 0 && 
                advert.routeCount < MAX_ADVERT_ROUTES) {
                advert.routes[advert.routeCount].dest = routes[i].dest;
                advert.routes[advert.routeCount].hops = routes[i].hops;
                advert.routeCount++;
            }
        }
        memcpy(pkt.payload, &advert, sizeof(HelloPayload));

        //radio.standby();
        delay(random(40, 200));

        if (channelBusy()) {
            delay(getBackoff(PRI_HELLO));
            return;
        }

        EncryptedMeshPacket encHello;
        if (!encryptMeshPacket(&pkt, &encHello)) {
            Serial.println("[CRYPTO] Encryption failed in sendHello!");
            return;
        }
        radio.transmit((uint8_t*)&encHello, sizeof(EncryptedMeshPacket));
        lastRXActivity = millis();
        delay(10);
        radio.setPacketReceivedAction(setFlag);
        int rxState = radio.startReceive();
        Serial.print("startReceive after HELLO: ");
        Serial.println(rxState);
        Serial.println("HELLO sent (encrypted)");
    }
}

void updateRoute(uint8_t dest, uint8_t nextHop, uint8_t hops, int16_t rssi) {
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == dest) {
            bool betterHopCount = hops < routes[i].hops;
            bool samePath = (nextHop == routes[i].nextHop);
            bool betterRSSI = rssi > routes[i].linkRSSI;

            if (betterHopCount || (samePath && betterRSSI)) {
                routes[i].nextHop = nextHop;
                routes[i].hops = hops;
                routes[i].linkRSSI = rssi;

                Serial.print(millis() / 1000);
                Serial.print("s Route added: dest=");
                Serial.print(dest);
                Serial.print(" via=");
                Serial.print(nextHop);
                Serial.print(" hops=");
                Serial.println(hops);
            }

            if (samePath || betterHopCount) {
                routes[i].lastUpdated = millis(); // refresh if route confirmed
            }
            return;
        }
    }
    // New route
    int worstIndex = -1;
    unsigned long oldest = ULONG_MAX;

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == 0) {
            worstIndex = i;
            break;
        }
    }

    if (worstIndex == -1) {
        for (int i = 0; i < MAX_ROUTES; i++) {
            if (routes[i].lastUpdated < oldest) {
                oldest = routes[i].lastUpdated;
                worstIndex = i;
            }
        }
    }

    if (worstIndex != -1) {
        routes[worstIndex].dest = dest;
        routes[worstIndex].nextHop = nextHop;
        routes[worstIndex].hops = hops;
        routes[worstIndex].lastUpdated = millis();
        routes[worstIndex].linkRSSI = rssi;

        Serial.print("Route added/replaced: dest=");
        Serial.print(dest);
        Serial.print(" via=");
        Serial.print(nextHop);
        Serial.print(" hops=");
        Serial.println(hops);
    }
}

void updateNeighbors(MeshPacket* pkt, int16_t rssi) {
    // Update neighbor table
    Serial.print("HELLO received from node ");
    Serial.println(pkt->src);
    bool found = false;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID == pkt->src) {
            neighbors[i].rssi = rssi;
            neighbors[i].lastSeen = millis();
            found = true;
            break;
        }
    }
    if (!found) {
        for (int i = 0; i < MAX_NEIGHBORS; i++) {
            if (neighbors[i].nodeID == 0) {
                neighbors[i].nodeID = pkt->src;
                neighbors[i].lastSeen = millis();
                neighbors[i].rssi = (int16_t)rssi;
                Serial.print("New neighbor: ");
                Serial.println(pkt->src);
                break;
            }
        }
    }

    // Always add direct 1-hop route to sender
    updateRoute(pkt->src, pkt->src, 1, (int16_t)rssi);

    // Parse and merge their advertised routes
    HelloPayload advert;
    memcpy(&advert, pkt->payload, sizeof(HelloPayload));

    for (int i = 0; i < advert.routeCount; i++) {
        uint8_t dest = advert.routes[i].dest;
        uint8_t hops = advert.routes[i].hops + 1;

        if (dest == NodeIdentity::getNodeId()) continue; // don't route to yourself

        updateRoute(dest, pkt->src, hops, (int16_t)rssi);
    }
}

void cleanStaleNeighbors() {
    const unsigned long NEIGHBOR_TIMEOUT = 45000;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID != 0 &&
            millis() - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT) {

            Serial.print("Neighbor expired: ");
            Serial.println(neighbors[i].nodeID);

            neighbors[i].nodeID = 0;
            neighbors[i].rssi = -200;
            neighbors[i].lastSeen = 0;
        }
    }
}

void cleanStaleRoutes() {
    const unsigned long ROUTE_EXPIRY = 45000;
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != 0 && 
            (millis() - routes[i].lastUpdated) > ROUTE_EXPIRY) {
            Serial.print(millis() / 1000);
            Serial.print("s Route expired: dest=");
            Serial.println(routes[i].dest);
            routes[i].dest = 0;
            routes[i].nextHop = 0;
            routes[i].hops = 0;
            routes[i].lastUpdated = 0;
            routes[i].linkRSSI = -128; 
        }
    }
}

void printRoutingTable() {
    Serial.println("=== Routing Table ===");
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != 0) {
            Serial.print("dest=");
            Serial.print(routes[i].dest);
            Serial.print(" nextHop=");
            Serial.print(routes[i].nextHop);
            Serial.print(" hops=");
            Serial.print(routes[i].hops);
            Serial.print(" age=");
            Serial.print((millis() - routes[i].lastUpdated) / 1000);
            Serial.println("s");
        }
    }
    Serial.println("=====================");
}

uint8_t selectNextHop(uint8_t finalDest, uint8_t srcNode) {
    const int16_t MIN_RSSI = -100;

    uint8_t bestNext = 0;
    int16_t bestRSSI = -999;
    uint8_t bestHops = 255;

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != finalDest) continue;

        uint8_t nextHop = routes[i].nextHop;
        if (nextHop == srcNode) continue;

        bool neighborFound = false;
        int16_t linkRSSI = routes[i].linkRSSI;

        for (int j = 0; j < MAX_NEIGHBORS; j++) {
            if (neighbors[j].nodeID == nextHop) {
                if (millis() - neighbors[j].lastSeen > 30000) {
                    Serial.println("Neighbor stale, using stored linkRSSI");
                    neighborFound = true;
                    break;
                }
                linkRSSI = neighbors[j].rssi;
                neighborFound = true;
                break;
            }
        }

        if (!neighborFound) continue;
        if (linkRSSI < MIN_RSSI) continue;

        if (linkRSSI > bestRSSI) {
            bestRSSI = linkRSSI;
            bestHops = routes[i].hops;
            bestNext = nextHop;
        } else if (abs(linkRSSI - bestRSSI) <= 2 && routes[i].hops < bestHops) {
            bestHops = routes[i].hops;
            bestNext = nextHop;
        }
    }

    return bestNext;
}

bool reliableTransmit(MeshPacket &pkt, uint8_t maxRetries) {
    for (int attempt = 0; attempt < maxRetries; attempt++) {

        if (channelBusy()) {
            delay(getBackoff(PRI_ACK));
        }

        EncryptedMeshPacket encPkt;
        if (!encryptMeshPacket(&pkt, &encPkt)) {
            Serial.println("[CRYPTO] Encryption failed in reliableTransmit!");
            return false;
        }
        int state = radio.transmit((uint8_t*)&encPkt, sizeof(EncryptedMeshPacket));

        if (state == RADIOLIB_ERR_NONE) {
            delay(10);
            radio.setPacketReceivedAction(setFlag);
            return true; // success
        }

        Serial.print("TX failed, retry ");
        Serial.println(attempt + 1);

        delay(50 + random(0, 50));
    }

    return false; // failed after retries
}

void handleTX() {
	buttonState = digitalRead(BUTTON_PIN);
    if (buttonState != lastButtonState) {
        if (buttonState == LOW) {

            if (pendingACK) {
                Serial.println("Waiting for ACK, ignoring button press");
                lastButtonState = buttonState;
                return;
            }

            Serial.println("Button Pressed Once!");
            display.setCursor(0,10); 
            display.print("TX: Packet sent"); 
            display.display(); 

            static uint16_t seqCounter = 0;

            MeshPacket pkt = {};
            pkt.src = NodeIdentity::getNodeId();
            pkt.lastHop = NodeIdentity::getNodeId();
            pkt.dst = 3;
            pkt.ttl = 5;
            pkt.seq = seqCounter++;

            pendingAckSeq = pkt.seq;
            ackReceivedFlag = false;
            ackWaitStart = millis();

            pkt.type = PKT_GPS;

            buildPacket(counter, 1, pkt.payload);
            counter++;

            /*
            // HARD CODE: send first hop to Node 2
            pkt.nextHop = 2;
            */

           pkt.nextHop = selectNextHop(pkt.dst, NodeIdentity::getNodeId());

            if (pkt.nextHop == 0) {
                Serial.println("No route yet, waiting for HELLO convergence...");
                lastButtonState = buttonState;
                return;
            }
            display.setCursor(0,20);
            display.print("packet dest: ");
            display.print(pkt.dst);
            display.setCursor(0,30);
            display.print("packet id: ");
            display.print(pkt.seq); 
            display.setCursor(0, 50);
            display.print("FWD: ");
            display.print(pkt.nextHop);
            display.display();

            delay(random(40, 200));
            radio.standby();

            if (channelBusy()) {
                Serial.println("GPS delayed due to busy channel");
                delay(getBackoff(PRI_GPS));
            }
            EncryptedMeshPacket encTx;
            if (!encryptMeshPacket(&pkt, &encTx)) {
                Serial.println("[CRYPTO] Encryption failed in handleTX!");
                lastButtonState = buttonState;
                return;
            }
            radio.transmit((uint8_t*)&encTx, sizeof(EncryptedMeshPacket));
            lastRXActivity = millis();
            delay(10);
            radio.setPacketReceivedAction(setFlag);
            radio.startReceive();

            pendingACK = true;
            pendingPkt = pkt;
            pendingSeq = pkt.seq;
            pendingSentAt = millis();
            pendingRetries = 0;
        }
        lastButtonState = buttonState;
    }
}

void handleRX() {
    if (!receivedFlag) return;
    Serial.println("handleRX triggered");
    receivedFlag = false;
    lastRXActivity = millis();

    uint8_t inBuf[sizeof(EncryptedMeshPacket)];
    int len = radio.getPacketLength();

    if (len != sizeof(EncryptedMeshPacket)) {
        Serial.print("[RX] Dropped bad len=");
        Serial.print(len);
        Serial.print(" expected=");
        Serial.println((int)sizeof(EncryptedMeshPacket));
        radio.startReceive();
        return;
    }

    int state = radio.readData(inBuf, len);

    float rssi = radio.getRSSI();
    float snr = radio.getSNR();

    Serial.print("PKT received type=");
    Serial.print(((MeshPacket*)inBuf)->type);
    Serial.print(" src=");
    Serial.print(((MeshPacket*)inBuf)->src);
    Serial.print(" dst=");
    Serial.println(((MeshPacket*)inBuf)->dst);

    if (state != RADIOLIB_ERR_NONE) {
        radio.startReceive();
        return;
    }

    EncryptedMeshPacket *encRx = (EncryptedMeshPacket*)inBuf;
    MeshPacket decPkt;
    if (!decryptMeshPacket(encRx, &decPkt)) {
        Serial.println("[CRYPTO] Decryption failed");
        radio.startReceive();
        return;
    }
    MeshPacket *pkt = &decPkt;

    Serial.print("RAW RX: type=");
    Serial.print(pkt->type);
    Serial.print(" src=");
    Serial.print(pkt->src);
    Serial.print(" dst=");
    Serial.println(pkt->dst);

    // Ignore own packets
    if (pkt->src == NodeIdentity::getNodeId()) {
        radio.startReceive();
        return;
    }

    /*
    // remove for outdoor testing
    /*
    #if NODE_ID == 1
        if (pkt->lastHop == 3) {
            radio.startReceive();
            return;
        }
    #endif
    #if NODE_ID == 3
        if (pkt->lastHop == 1) {
            radio.startReceive();
            return;
        }
    #endif
    */    
  
    //stop if the packet already been seen before
    /*
    bool isDuplicate = seenBefore(pkt->src, pkt->seq);

    if (pkt->type != PKT_HELLO && isDuplicate) {
        radio.startReceive();
        return;
    }
    */


    /*
    if (pkt->type != PKT_HELLO && hasSeen(pkt->src, pkt->seq)) {
        Serial.print("Dropped duplicate src=");
        Serial.print(pkt->src);
        Serial.print(" seq=");
        Serial.println(pkt->seq);
        radio.startReceive();
        return;
    }
    */

    bool isDuplicate = hasSeen(pkt->src, pkt->seq);

    if (pkt->type != PKT_HELLO && isDuplicate) {

        if (pkt->dst == NodeIdentity::getNodeId()) {
            Serial.println("Duplicate received at destination → re-sending ACK");

        } else {
            Serial.print("Dropped duplicate src=");
            Serial.print(pkt->src);
            Serial.print(" seq=");
            Serial.println(pkt->seq);
            radio.startReceive();
            return;
        }
    }

    // HELLO packets update neighbors immediately
    if (pkt->type == PKT_HELLO) {
        Serial.println("Hello Received"); 
        updateNeighbors(pkt, (int16_t)rssi);
        radio.startReceive();
        return;
    }

    if (pkt->type == PKT_ACK && pkt->dst == NodeIdentity::getNodeId()) {
        ACKPayload ackPayload;
        memcpy(&ackPayload, pkt->payload, sizeof(ACKPayload));
        Serial.print("ACK received for seq=");
        Serial.println(ackPayload.originalSeq);

        if (pendingACK && ackPayload.originalSeq == pendingSeq) {
            pendingACK = false;
            Serial.println("Pending ACK cleared");
        }

        display.fillRect(0, 40, 128, 24, SSD1306_BLACK);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0, 40);
        display.print("ACK received!");
        display.setCursor(0, 50);
        display.print("seq=");
        display.print(ackPayload.originalSeq);
        display.display();
        radio.startReceive();
        return;
    }

    if (pkt->type == PKT_ACK && pkt->dst == NODE_ID) {
        ACKPayload ackPayload;
        memcpy(&ackPayload, pkt->payload, sizeof(ACKPayload));

        Serial.print("ACK received for seq=");
        Serial.println(ackPayload.originalSeq);

        display.fillRect(0, 40, 128, 24, SSD1306_BLACK);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0, 40);
        display.print("ACK received!");
        display.setCursor(0, 50);
        display.print("seq=");
        display.print(ackPayload.originalSeq);
        display.display();

        radio.startReceive();
        return;
    }

    // DEBUG: show what this node received
    if (pkt->type == PKT_GPS) {
        Serial.print("RX from node ");
        Serial.print(pkt->src);
        Serial.print(" lastHop=");
        Serial.println(pkt->lastHop);

        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0,10); 
        display.print("RSSI: ");
        display.print((int)rssi);
        display.print(" ");
        display.print("SNR: "); 
        display.print(snr, 1); 
        display.setCursor(0,20);
        display.print("dest: ");
        display.print(pkt->dst);
        display.print(" "); 
        display.print("pkt id: ");
        display.print(pkt->seq); 
        display.setCursor(0,30);
        display.print("next hop: "); 
        display.print(pkt->nextHop); 
        display.display(); 


    }

    // Packet reached final destination
    if (pkt->type == PKT_GPS && pkt->dst == NodeIdentity::getNodeId()) {
        float rxLat, rxLng;
        uint8_t rxNode;
        parsePacket(pkt->payload, rxLat, rxLng, rxNode);

        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0, 40);
        display.print("RX ");
        display.print(pkt->src);
        display.print(": ");
        display.print(rxLat, 4);
        display.print(",");
        display.print(rxLng, 4);

        display.setCursor(50, 50);
        display.print("Last Hop: ");
        display.print(pkt->lastHop); 
        display.display();

        // Build and send ACK
        MeshPacket ack = {};
        ack.src = NODE_ID;
        ack.lastHop = NODE_ID;
        ack.dst = pkt->src;
        ack.ttl = 5;
        ack.type = PKT_ACK;

        static uint16_t ackSeq = 0;
        ack.seq = ackSeq++;

        ACKPayload ackPayload = {};
        ackPayload.originalSrc = pkt->src;
        ackPayload.originalSeq = pkt->seq;
        memcpy(ack.payload, &ackPayload, sizeof(ACKPayload));

        ack.nextHop = selectNextHop(ack.dst, NODE_ID);
        if (ack.nextHop != 0) {
            Serial.print("Sending ACK to node ");
            Serial.println(ack.dst);
            delay(random(20, 80));
            radio.standby();
            radio.transmit((uint8_t*)&ack, sizeof(MeshPacket));
        }
        radio.startReceive();
        return;
    }

    // Forward if not final destination
    if ((pkt->type == PKT_GPS || pkt->type == PKT_ACK) && pkt->dst != NODE_ID && pkt->ttl > 0) {

        // Only forward if this node is the intended nextHop
        if (pkt->nextHop == NodeIdentity::getNodeId() || pkt->nextHop == MESH_BROADCAST) {

            if (pkt->lastHop == NodeIdentity::getNodeId()) return;

            /*
            // HARDCODE FOR PROOF OF CONCEPT:
            // Node 1 → 2 → 3
            if (NODE_ID == 2) {
                pkt->nextHop = 3;
            }
            */

           pkt->nextHop = selectNextHop(pkt->dst, pkt->src);
            if (pkt->nextHop == 0) {
                radio.startReceive();
                return; // no route, drop it
            }

            // Update lastHop BEFORE forwarding
            pkt->lastHop = NodeIdentity::getNodeId();
            pkt->ttl--;

            Serial.print("Forwarding ");
            Serial.print(pkt->type == PKT_ACK ? "ACK" : "packet");
            Serial.print(" to node: ");
            Serial.println(pkt->nextHop);

            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            display.setCursor(0, 50);
            display.print("FWD: ");
            display.print(pkt->nextHop);
            display.display();

            delay(random(40, 200));
            //radio.standby(); 

            int attempts = 0;
            while (channelBusy() && attempts < 3) {
                if (pkt->type == PKT_ACK) {
                    delay(getBackoff(PRI_ACK));
                } else {
                    delay(getBackoff(PRI_GPS));
                }
                attempts++;
            }

            EncryptedMeshPacket fwdPkt;
            if (!encryptMeshPacket(pkt, &fwdPkt)) {
                Serial.println("[CRYPTO] Re-encryption failed in forward!");
                radio.startReceive();
                return;
            }
            radio.transmit((uint8_t*)&fwdPkt, sizeof(EncryptedMeshPacket));
            markSeen(pkt->src, pkt->seq); 
            lastRXActivity = millis();
            delay(10);
            radio.setPacketReceivedAction(setFlag);
            radio.startReceive();
        }
    }

    radio.startReceive();
}

void setup(){
    Serial.begin(9600); 
	//delay(1000); 
	pinMode(LED, OUTPUT); //set output mode
	pinMode(BUTTON_PIN, INPUT_PULLUP);

#ifdef _VARIANT_PROMICRO_V2_DIY_
	// give powah
	pinMode(PIN_GPS_EN, OUTPUT);
	digitalWrite(PIN_GPS_EN, HIGH);
	digitalWrite(PIN_DISPLAY_EN, HIGH);
	delay(250);
#endif // _VARIANT_PROMICRO_V2_DIY_

	digitalWrite(LED, LOW);

	Wire.begin();
	SPI.begin();

	for(int i = 0; i < MAX_NEIGHBORS; i++){
        neighbors[i].nodeID = 0;
        neighbors[i].lastSeen = 0;
        neighbors[i].rssi = -200;
    }

	for(int i = 0; i < MAX_ROUTES; i++){
    	routes[i].dest = 0;
        routes[i].nextHop = 0;
        routes[i].hops = 0;    
        routes[i].lastUpdated = 0; 
        routes[i].linkRSSI = -128;
	}
	
	gpsSerial.begin(9600);
	//SPI.begin(); // uses variant default SPI pins

	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { //if error then builtin led light up
	  digitalWrite(LED, HIGH); //Set the LED to high
	  while (true);
	}

	if (!initCrypto()) {
	  Serial.println("[CRYPTO] Initialization failed!");
	  digitalWrite(LED, HIGH);
	  while (true) { delay(1000); }
	} /*
	if (!testCrypto()) {
	  Serial.println("[CRYPTO] Self-tests failed!");
	  digitalWrite(LED, HIGH);
	  while (true) { delay(1000); }
	}
	measureCryptoLatency();
    */ 
	
	// display boot
	memset(buffer, 0, sizeof(buffer)); // set loop display buffer with zeros
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("Booting or smth...");
	display.display();

	// modem init
	// initialize SX1262 with default settings
	Serial.print(F("[SX1262] Initializing ... "));
	int state = radio.begin();
	if (state == RADIOLIB_ERR_NONE) {
	  Serial.println(F("success!"));
	} else {
	  Serial.print(F("failed, code "));
	  Serial.println(state);
	  while (true) { delay(10); 
	  Serial.println("radio errored");}
	}

	// set modem parameters
	radio.setFrequency(FREQ_MHZ);
	radio.setSpreadingFactor(SF);
	radio.setBandwidth(BW);
	radio.setCodingRate(CR);
	radio.setPreambleLength(PREAMBLE);
	radio.setSyncWord(SYNCWORD);
	//radio.setCRC(2);
	radio.setPacketReceivedAction(setFlag); // IRQ func
	
	// start listening for LoRa packets
	Serial.print(F("[SX1262] Starting to listen ... "));
	state = radio.startReceive();
	if (state == RADIOLIB_ERR_NONE) {
	  Serial.println(F("success!"));
	} else {
	  Serial.print(F("failed, code "));
	  Serial.println(state);
	  while (true) { delay(10); }
	}
	Serial.println("boot successful...");
	
	delay(1000);
	display.clearDisplay();
	display.display();
	
    /*
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // White text on black background (for overwr)
    /*
	display.setCursor(0, 0);
	display.print("Lat: NONE");
	display.setCursor(0, 10);
	display.print("Lng: NONE");
	display.setCursor(0, 20);
	display.print("Date: NONE");
    */
	display.setCursor(0, 0);
	display.print("Node ID: ");
	display.print(NODE_ID); 
	display.display();

    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print(NodeIdentity::getNodeName());
    if (NodeIdentity::isHeadNode()) {
        display.print(" HEAD-NODE");
    }
    display.display();

    delay(random(1000, 4000));

}

void loop() {
	const size_t BUF_SZ = 256;

    // Read and decode GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
        lastHeartbeat = millis();
        Serial.print("[ALIVE] uptime=");
        Serial.print(millis() / 1000);
        Serial.println("s");
    }

    //Checking that packets are actually being received
    if (millis() - lastRXActivity > 55000) {
        Serial.println("WATCHDOG: restarting RX mode");
        radio.setPacketReceivedAction(setFlag); 
        radio.startReceive();
        lastRXActivity = millis();
    }

    if (pendingACK && millis() - pendingSentAt > ACK_TIMEOUT) {
        if (pendingRetries < MAX_RETRIES) {
            pendingRetries++;
            Serial.print("ACK timeout - retry ");
            Serial.print(pendingRetries);
            Serial.print("/");
            Serial.println(MAX_RETRIES);

            pendingPkt.nextHop = selectNextHop(pendingPkt.dst, NodeIdentity::getNodeId());
            pendingPkt.lastHop = NodeIdentity::getNodeId();

            if (pendingPkt.nextHop == 0) {
                Serial.println("No route yet, will retry again");
                pendingSentAt = millis();
                pendingRetries--;
            } else {
                display.fillRect(0, 40, 128, 24, SSD1306_BLACK);
                display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
                display.setCursor(0, 40);
                display.print("Retry ");
                display.print(pendingRetries);
                display.print("/");
                display.print(MAX_RETRIES);
                display.display();

                if (channelBusy()) delay(getBackoff(PRI_GPS));
                radio.standby();
                EncryptedMeshPacket encRetry;
                if (!encryptMeshPacket(&pendingPkt, &encRetry)) {
                    Serial.println("[CRYPTO] Encryption failed in retry!");
                    pendingSentAt = millis();
                    pendingRetries--;
                    radio.startReceive();
                    return;
                }
                radio.transmit((uint8_t*)&encRetry, sizeof(EncryptedMeshPacket));
                lastRXActivity = millis();
                delay(10);
                radio.setPacketReceivedAction(setFlag);
                radio.startReceive();
                pendingSentAt = millis();
            }
        } else {
            Serial.println("Max retries reached - SEND FAILED");
            pendingACK = false;
            display.fillRect(0, 40, 128, 24, SSD1306_BLACK);
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            display.setCursor(0, 40);
            display.print("SEND FAILED");
            display.display();
        }
    }

    //bool locationUpdated = gps.location.isUpdated();
    bool dateUpdated = gps.date.isUpdated();

	sendHello(); 
	//readGPS(); 

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 20000) {
        lastPrint = millis();
        printRoutingTable();
    }

    static unsigned long lastCleanup = 0;
    if (millis() - lastCleanup > 15000) {
        lastCleanup = millis();
        cleanStaleRoutes();
    }

    static unsigned long lastNeighborCleanup = 0;
    if (millis() - lastNeighborCleanup > 15000) {
        lastNeighborCleanup = millis();
        cleanStaleNeighbors();
    }
 
    /*
    if (dateUpdated) {
        TinyGPSDate date = gps.date;

        if (date.isValid() && date.value() != lastDate.value()) {
            lastDate = date;
            snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d", date.year(), date.month(), date.day());

            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Set color for date (also for overwr)
            display.setCursor(0, 20); // curosr pos for date
            display.print(dateBuffer);

            display.display();
        }
    }
    */



	/*
	Serial.println("Current neighbors:");
	for(int i = 0; i < MAX_NEIGHBORS; i++){
		if(neighbors[i].nodeID != 0){
			Serial.print("  Node ");
			Serial.print(neighbors[i].nodeID);
			Serial.print(" lastSeen=");
			Serial.println(neighbors[i].lastSeen);
		}
	}
	Serial.println("---");
    */

	
	
	handleRX(); 
	handleTX(); 

}


// Convert bytes to ASCII-safe String
String bytesToAscii(const uint8_t *buf, size_t len) {
  String s;
  s.reserve(len);
  for (size_t i = 0; i < len; i++) {
    char c = (char)buf[i];
    if (c >= 32 && c <= 126) {
      s += c;            // printable ASCII
    } else {
      s += '.';          // non-printable → dot
    }
  }
  return s;
}
