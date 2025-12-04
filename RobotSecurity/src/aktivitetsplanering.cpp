#include "aktivitetsplanering.h"
#include "test.h"
#include "position.hpp"
#include <queue>
#include <limits>
#include "Communication.h"
#include "Detection.h"


#define TRIG_PIN 13
#define ECHO_PIN 14
#define RADAR_RX_PIN 17
#define RADAR_TX_PIN 16

UltrasonicSensor ultrasonicSensor(TRIG_PIN,ECHO_PIN);
RadarSensor radar(Serial1,RADAR_RX_PIN,RADAR_TX_PIN);

// real definitions of globals
int counter = 0;
int battery = 0;
int msgCount = 0;

nodeRobot currentRobot;
std::vector<Vertex> nodeList;//positioner
std::vector<nodeRobot> robotList;//kopplade robbotar
bool isMaster = false;
String currentInstruktions;


/*
1.assume master
2.Skapa en lista med instructioner till alla robotar(bara en robot nu)
3.följ instruktioner medan den aktiverar update metoden varje 5s
4.När instruktioner klar, aktivera klar metoden och vänta på instruktion
5.gå tillbaka till steg 2 eftersom den master
*/

void setupVertexes()
{
    nodeList = 
    {
    {15.72f, 40.74f, {}, {}, {}, 0, false, 0},
    {15.72f, 38.46f, {}, {}, {}, 1, false, 1},
    {21.84f, 35.04f, {}, {}, {}, 1, false, 2},
    {21.72f, 31.26f, {}, {}, {}, 1, false, 3},
    {21.78f, 26.94f, {}, {}, {}, 1, false, 4},
    {21.72f, 22.56f, {}, {}, {}, 1, false, 5},
    {21.78f, 18.54f, {}, {}, {}, 1, false, 6},
    {21.72f, 14.40f, {}, {}, {}, 1, false, 7},
    {21.78f, 10.32f, {}, {}, {}, 1, false, 8},
    {15.84f,  5.04f, {}, {}, {}, 1, false, 9},
    {10.14f,  9.78f, {}, {}, {}, 1, false, 10},
    {10.14f, 14.82f, {}, {}, {}, 1, false, 11},
    {10.08f, 18.90f, {}, {}, {}, 1, false, 12},
    {10.14f, 23.04f, {}, {}, {}, 1, false, 13},
    {10.02f, 27.36f, {}, {}, {}, 1, false, 14},
    {10.14f, 31.26f, {}, {}, {}, 1, false, 15},
    {10.02f, 35.58f, {}, {}, {}, 1, false, 16},
    {15.72f, 23.16f, {}, {}, {}, 1, false, 17},
    {15.84f, 50.28f, {}, {}, {}, 0, false, 18},
    {15.84f, 48.00f, {}, {}, {}, 2, false, 19},
    {21.60f, 43.68f, {}, {}, {}, 2, false, 20},
    {15.72f, 42.12f, {}, {}, {}, 2, false, 21},
    { 8.28f, 43.98f, {}, {}, {}, 2, false, 22},
    {14.52f, 91.74f, {}, {}, {}, 0, false, 23},
    {14.52f, 87.30f, {}, {}, {}, 3, false, 24},
    {30.42f, 85.98f, {}, {}, {}, 3, false, 25},
    {29.40f, 78.66f, {}, {}, {}, 3, false, 26},
    {32.34f, 78.54f, {}, {}, {}, 0, false, 27},
    {29.22f, 74.22f, {}, {}, {}, 3, false, 28},
    {30.30f, 70.50f, {}, {}, {}, 3, false, 29},
    {30.30f, 66.90f, {}, {}, {}, 3, false, 30},
    {30.30f, 62.70f, {}, {}, {}, 3, false, 31},
    {30.30f, 58.50f, {}, {}, {}, 3, false, 32},
    {30.30f, 54.30f, {}, {}, {}, 3, false, 33},
    {24.00f, 52.32f, {}, {}, {}, 3, false, 34},
    {15.84f, 52.56f, {}, {}, {}, 3, false, 35},
    {10.20f, 54.78f, {}, {}, {}, 3, false, 36},
    {10.20f, 58.68f, {}, {}, {}, 3, false, 37},
    {10.20f, 62.88f, {}, {}, {}, 3, false, 38},
    {10.20f, 66.48f, {}, {}, {}, 3, false, 39},
    {10.20f, 70.80f, {}, {}, {}, 3, false, 40},
    {10.20f, 74.58f, {}, {}, {}, 3, false, 41},
    {10.20f, 78.72f, {}, {}, {}, 3, false, 42},
    {10.20f, 82.62f, {}, {}, {}, 3, false, 43},
    {10.20f, 86.82f, {}, {}, {}, 3, false, 44},
    {21.06f, 78.66f, {}, {}, {}, 3, false, 45},
    {18.60f, 65.10f, {}, {}, {}, 3, false, 46}
    };

    auto link = [&](int from, int to, float dist, float head){
    nodeList[from].neighbors.push_back(&nodeList[to]);
    nodeList[from].distance.push_back(dist);
    nodeList[from].heading.push_back(head);
};

// node1
link(0, 1, 2.28f, 90.f);
link(0, 21, 1.38f, 270.f);

// node2
link(1, 0, 2.28f, 270.f);
link(1, 2, 7.0108f, 29.20f);

// node3
link(2, 3, 3.7819f, 91.82f);

// node4
link(3, 4, 4.3204f, 89.20f);

// node5
link(4, 5, 4.3804f, 90.78f);

// node6
link(5, 6, 4.0204f, 89.14f);

// node7
link(6, 7, 4.1404f, 90.83f);

// node8
link(7, 8, 4.0804f, 89.16f);

// node9
link(8, 9, 7.3235f, 234.38f);

// node10
link(9, 10, 7.2640f, 318.75f);
link(9, 17, 18.1200f, 92.38f);

// node11
link(10, 11, 5.04f, 90.f);

// node12
link(11, 12, 4.08f, 90.f);

// node13
link(12, 13, 4.14f, 90.f);

// node14
link(13, 14, 4.32f, 90.f);

// node15
link(14, 15, 3.9f, 90.f);

// node16
link(15, 16, 4.32f, 90.f);

// node17
link(16, 1, 7.0108f, 329.20f);

// node18
link(17, 1, 15.30f, 0.f);

// node19
link(18, 19, 2.28f, 90.f);
link(18, 35, 2.28f, 270.f);

// node20
link(19, 18, 2.28f, 270.f);
link(19, 20, 6.1506f, 329.41f);
link(19, 21, 5.88f, 270.f);

// node21
link(20, 21, 6.0f, 250.59f);

// node22
link(21, 19, 5.88f, 90.f);
link(21, 22, 8.0508f, 199.24f);
link(21, 0, 1.38f, 90.f);

// node23
link(22, 19, 8.0508f, 19.24f);

// node24
link(23, 24, 4.44f, 270.f);

// node25
link(24, 23, 4.44f, 90.f);
link(24, 25, 16.0679f, 347.38f);
link(24, 45, 9.0201f, 307.12f);

// node26
link(25, 26, 7.0724f, 251.41f);

// node27
link(26, 27, 2.9449f, 352.06f);
link(26, 28, 4.4479f, 270.34f);
link(26, 45, 8.3666f, 346.28f);

// node28
link(27, 26, 2.9449f, 172.06f);

// node29
link(28, 29, 3.8359f, 323.46f);

// node30
link(29, 12, 22.635f, 212.09f);

// node31
link(30, 22, 23.6376f, 215.94f);

// node32
link(31, 32, 4.2f, 270.f);

// node33
link(32, 33, 4.2f, 270.f);

// node34
link(33, 34, 6.3544f, 211.15f);

// node35
link(34, 35, 8.1607f, 242.07f);

// node36
link(35, 36, 5.945f, 252.43f);
link(35, 46, 13.9979f, 346.92f);
link(35, 18, 2.28f, 90.f);

// node37
link(36, 37, 3.9f, 270.f);

// node38
link(37, 38, 4.2f, 270.f);

// node39
link(38, 39, 3.6f, 270.f);

// node40
link(39, 13, 0.72f, 315.f);

// node41
link(40, 41, 3.78f, 270.f);

// node42
link(41, 33, 28.5533f, 45.26f);

// node43
link(42, 43, 3.9f, 270.f);

// node44
link(43, 44, 4.2f, 270.f);

// node45
link(44, 24, 4.3466f, 353.66f);

// node46
link(45, 24, 10.8361f, 232.88f);
link(45, 26, 8.34f, 0.f);
link(45, 46, 13.7813f, 100.28f);

// node47
link(46, 45, 13.7813f, 280.28f);
link(46, 35, 12.8401f, 102.41f);


}

void broadcast(Message message) {
    // Förmedla position och ID

    //sendMessage(getPosition(), getMacAddress());

}


void sendMessageToServer() {
    // Skickar position/ID
    // Inkludera sig själv som master

}


Message receiveMessage() {

    Message temp;
    temp.messageId = 0;
    temp.messsage = "din mam";
    temp.time = 0;
    return temp;
    // Tar emot meddelande
    // Tar emot position/ID
    // Tar emot kommandon från master
    // Unika robotID och vem som är master
    
}


void moveTo(Vertex from, Vertex to) {
    // Tar emot W/A/S/D eller liknande styrkommandon
}


std::vector<Vertex*> navigate(Vertex* start, Vertex* goal) {
    const int N = static_cast<int>(nodeList.size());
    const float INF = std::numeric_limits<float>::infinity();

    // Omvandla start/goal-pekare till index i nodeList
    int startIndex = static_cast<int>(start - &nodeList[0]);
    int goalIndex  = static_cast<int>(goal  - &nodeList[0]);

    // Dijkstra-strukturer
    std::vector<float> dist(N, INF);
    std::vector<int>   prev(N, -1);
    std::vector<bool>  visited(N, false);

    using NodeState = std::pair<float, int>; // (dist, index)
    std::priority_queue<
        NodeState,
        std::vector<NodeState>,
        std::greater<NodeState>
    > pq;

    dist[startIndex] = 0.0f;
    pq.push({0.0f, startIndex});

    // Dijkstra
    while (!pq.empty()) {
        auto top = pq.top();
        float d = top.first;
        int   u = top.second;

        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;

        if (u == goalIndex) break;  // vi är framme

        Vertex& vtx = nodeList[u];

        // gå igenom alla grannar
        for (size_t k = 0; k < vtx.neighbors.size(); ++k) {
            Vertex* neigh = vtx.neighbors[k];
            float   w     = vtx.distance[k];  // kostnad till den grannen

            int v = static_cast<int>(neigh - &nodeList[0]); // index

            float newDist = d + w;
            if (newDist < dist[v]) {
                dist[v] = newDist;
                prev[v] = u;
                pq.push({newDist, v});
            }
        }
    }

    // Återskapa väg från goalIndex tillbaka till startIndex
    std::vector<int> indexPath;
    for (int u = goalIndex; u != -1; u = prev[u]) {
        indexPath.push_back(u);
    }
    std::reverse(indexPath.begin(), indexPath.end());

    // Bygg returvektor av Vertex*
    std::vector<Vertex*> path;
    path.reserve(indexPath.size());
    for (int idx : indexPath) {
        path.push_back(&nodeList[idx]);
    }

    return path;
}


String getMac() {
    return WiFi.macAddress();
}







void startNetwork()
{

    
    //xTaskCreatePinnedToCore(runningUpdateMessages, "runningUpdateMessages", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(runningUpdateMessages, "runningUpdateMessages", 4096, NULL, 1, NULL, 1);  //För att kontinuerligt skicka updates
    xTaskCreatePinnedToCore(runningReceiveMessages, "runningReceiveMessages", 4096, NULL, 1, NULL, 1);  //För att kontinuerligt ta emot medelanden

}


void runningUpdateMessages(void * parameter)
{
  while (true)
  {
    
    Serial.println(createUpdate());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
    vTaskDelete(NULL);
}


void runningReceiveMessages(void * parameter)
{
  while (true)
  {
    
    dealWithMessage(receiveMessage());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
    vTaskDelete(NULL);
}

// Skapar instruktioner för alla robotar-------------------------------
String plan() {
    
    std::vector<nodeRobot> avelableRobots;
    std::vector<int> roomsBeingChecked;

    //kollar vilka robotar som är lediga
    for (int i = 0; i < robotList.size(); i++)
    {
        if(robotList[i].taskedRoom == 0)
        {
            avelableRobots.push_back(robotList[i]);
            
        }
        else
        {
            roomsBeingChecked.push_back(robotList[i].taskedRoom);
        }
    }

    //kollar vilka rum som saknar robotar och deligerar lediga robotar till dessa rum.
    for(int i = 0; i < nodeList.size(); i++)
    {
        if(!nodeList[i].checked && !avelableRobots.empty() && (std::find(roomsBeingChecked.begin(), roomsBeingChecked.end(), nodeList[i].room) == roomsBeingChecked.end()))
        {
            avelableRobots.back().taskedRoom = nodeList[i].room;
            roomsBeingChecked.push_back(nodeList[i].room);
            avelableRobots.pop_back();

        }
    }

    String instructions = "<instructions>";
    
    for(int i = 0; i < robotList.size(); i++)
    {
        instructions = instructions + ":1:" + robotList[i].macAddress + ":2:" + robotList[i].taskedRoom + ":3:";
    }

    currentInstruktions = instructions;
    return instructions;
}


int createMessageId() {
    
    for (int i = 0; i < robotList.size(); i++)
    {
        
    }
    
   return 0;
}




String createUpdate() {
    //Create message updating current state, currently only pos
    //String message = "<uppdate>|:pos:" + String(currentRobot.pos->x) + "," + String(currentRobot.pos->y) + ":currentVertex:" + currentRobot.currentVertex.id + ":battery:" + String(checkBattery()) + "|";
    String message = "din mam";
    return message;
}


String createConnectionMessage()
{
    //Create message informing everyone about your connections and whos master.
    String message = "Connection";
    return message;
}


void dealWithMessage(Message msg)
{
    String message = msg.messsage;

    if (isMaster)
    {
        if(message.indexOf("<uppdate>") > -1)
        {
            Serial.println("Master: " + message);
        }
        else if(message.indexOf("<instructions>") > -1)
        {
            Serial.println("Master: " + message);

            
            
            int lastInstanceIndex = 0;
            while(message.indexOf(":1:", lastInstanceIndex) != -1)
            {
                int beginning = message.indexOf(":1:", lastInstanceIndex)+3;
                int end = message.indexOf(":2:", message.indexOf(":1:", lastInstanceIndex) + 3);
                
                message.substring(beginning, end);

                lastInstanceIndex = end;
                
            }
                
        }


        //Fixaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa

    }
    else
    {
        Serial.println("slave: " + message);
    }
}


void run() {
  // Huvudlogik / FSM
  // While-loop eller liknande
  // T.ex. olika logik om master/slave

  createMaster();
  startNetwork();

  while(isMaster) {

    Message temp;
    temp.messsage = plan();
    //skicka till alla robotar
    dealWithMessage(temp);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }

  while (!isMaster)
  {
    createUpdate();
    Message message;
    message.time = millis();
    message.messageId = createMessageId();
    message.messsage = createUpdate();
    broadcast(message);

    for (int i = msgCount; i > 0; i--)
    {
        dealWithMessage(receiveMessage());
    }
  }
}


void setupNetwork()
{

    std::vector<String> macList;

    macList.push_back("02:1A:7C:9F:34:B2");
    macList.push_back("02:1A:7C:9F:34:B3");
    macList.push_back("02:1A:7C:9F:34:B4");
    macList.push_back("02:1A:7C:9F:34:B5");
    macList.push_back("02:1A:7C:9F:34:B6");
    macList.push_back("02:1A:7C:9F:34:B7");
    macList.push_back("02:1A:7C:9F:34:B8");
    macList.push_back("02:1A:7C:9F:34:B9");

    
}

void detectionSetUp(){
    ultrasonicSensor.begin();
    radar.begin();
}
void setUp() {
    // Initial setup-logik
    currentRobot.isMaster = false;
    currentRobot.macAddress = getMac();

    setupVertexes();
    currentRobot.currentVertex = nodeList[0];
    robotList.push_back(currentRobot);


    detectionSetUp();


    run();

    // Sortera meddelanden, bygg listor, osv
}


//fixa senare
void createMaster() {
    // Skapa / välj master
    isMaster = true;
}


void charge() {
    // Laddlogik
}


void createsChargeSchedule() {
    // Master skapar laddschema
}


int checkBattery() {
    return 0;
    // Kontrollera batterinivå
}

void alarm() {
    // Kalla på vakt / larma
}

bool detectMovement() {
    // Detektera movement på marken och i rummet
    //true == movement, false = clear
    return radar.detectMovement();
}

void measureDistance() {
    // Detektera objekt som fönster och tavlor
    float distance = ultrasonicSensor.measureDistance();
    if(distance >= 15){
        //larma samt logga 
        Serial.printf("Object saknas/dörr öppen/fönster öppne");
    }else{
        Serial.printf("We good");
    }
}

void doTask()
{

}
