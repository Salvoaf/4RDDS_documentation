# Tutorial per lo sviluppatore
## Indice dei contenuti
1. [Componenti principali dell'ambiente 4DDS](#componenti-principali-dellambiente-4dds)
2. [PX4](#px4)
   - [Droni Testati](#droni-testati)
3. [ROS2](#ros2)
4. [Gazebo](#gazebo)
   - [Droni casuali](#droni-casuali)
   - [Guida per Estendere Gazebo con Sensori Dinamici](#guida-per-estendere-gazebo-con-sensori-dinamici)
   - [Guida per creare nuovi Worlds in Gazebo](#guida-per-creare-nuovi-worlds-in-gazebo)
5. [Problemi comuni](#problemi-comuni)
 
## Componenti principali dell'ambiente 4DDS

1. **PX4**: Autopilota dei droni. Modificato per aggiungere sensori mancanti e accettare setpoint di velocità in coordinate NED.

2. **ROS2**: Middleware per il controllo offboard e la logica di alto livello del drone.

   - Package principali:
     - `drone`: logica di un singolo drone.
     - `base_station`: coordina il flocking.
     - `obstacle_avoidance`: gestione degli ostacoli.
     - `target_detection`: rilevamento dei target tramite Hough Transform.
     - `parameters`: lettura parametri da file YAML.
     - `fdds_messages`: definizioni dei messaggi GeoPing e Flocking.
     - `analysis`: analisi dei log tramite `PathView.py`.

3. **Gazebo Classic**: Versione vecchia di Gazebo, integrata con PX4 per le simulazioni multi-drone e marine.

   - Modifiche per gestire il LIDAR e la telecamera dinamicamente, utilizzando più sensori dello stesso tipo.

## PX4

### Droni Testati
Clicca sul nome dei droni per avere una panormaica più chiara delle loro caratteristiche
#### Droni Aerei
   - [Iris](px4/Droni/Aria/iris.md)
#### Droni Terrestri
   - [Rover](px4/Droni/Terra/Rover.md)
#### Droni Acquatici
   - [BlueROV2](px4/Droni/Acqua/BlueROV2.md)







### Modifiche ai moduli di controllo per Rover e BlueROV2

#### **Rover**
- **Percorso file**: `/home/fourdds/PX4-Autopilot/src/modules/rover_pos_control/RoverPositionControl.cpp`

Il modulo di controllo per il rover è stato modificato per accettare setpoint di velocità in coordinate NED (North-East-Down), mentre in precedenza accettava solo coordinate relative al corpo del veicolo. Questo miglioramento consente un controllo più preciso e intuitivo rispetto al movimento globale del rover.

- **Funzione coinvolta**: `control_velocity`  
  Questa funzione è stata aggiornata per gestire i setpoint di velocità NED, migliorando la capacità del rover di muoversi nello spazio in base a coordinate geografiche piuttosto che in base al suo sistema di riferimento locale.

#### **BlueROV2**
- **Percorso file controllo dell'attitudine (roll, pitch, yaw)**: `/home/fourdds/PX4-Autopilot/src/modules/uuv_att_control/uuv_att_control.cpp`
- **Percorso file controllo della velocità e profondità**: `/home/fourdds/PX4-Autopilot/src/modules/uuv_pos_control/uuv_pos_control.cpp`

Il modulo di controllo della velocità e della profondità per il BlueROV2 è stato completamente riscritto. Questo aggiornamento garantisce una maggiore precisione nel controllo del movimento del veicolo subacqueo in termini di velocità e profondità.

- **Controllo dell'attitudine**: Il controllo di roll, pitch e yaw avviene nel file `uuv_att_control.cpp`, garantendo una gestione ottimale della stabilità e dell'orientamento del veicolo mentre opera sott'acqua.

- **Controllo della velocità e della profondità**:  
  La gestione della velocità e della profondità è implementata nel file `uuv_pos_control.cpp`. Questa riscrittura del modulo permette al BlueROV2 di muoversi con precisione lungo la coordinata Z, dove le coordinate NED (North-East-Down) restano il riferimento. Nella configurazione NED, le coordinate Z positive corrispondono a una maggiore profondità rispetto alla posizione di spawn del veicolo.

  Questa impostazione risulta cruciale per le missioni subacquee, dove è fondamentale mantenere un controllo accurato della profondità e della velocità lungo l'asse verticale.

---

Queste modifiche complessive ai moduli di controllo migliorano la capacità del rover di operare in ambienti terrestri e del BlueROV2 di operare in ambienti subacquei, consentendo movimenti più fluidi e precisi attraverso l'uso delle coordinate NED come riferimento.

## ROS2

ROS2 si occupa della logica di un singolo drone e di altri agenti autonomi, come la base station, rendendo il codice il più modulare possibile. I package ROS2 presenti in `/home/fourdds/ros2_ws/src` che contengono logiche già pronte sono i seguenti:

**Struttura dei package:**

- **drone:** Logica del singolo drone (armamento, decollo, flocking).
- **base_station:** Coordina lo sciame e gestisce il flocking.
- **obstacle_avoidance:** Gestisce l'evitamento degli ostacoli.
- **target_detection:** Rileva i target utilizzando tecniche come la trasformata di Hough.
- **parameters:** Gestisce i parametri di simulazione da un file YAML.
- **fdds_messages:** Definisce i messaggi utilizzati per la comunicazione.
- **analysis:** Contiene strumenti per l'analisi dei dati (es. `PathView.py`).


### **drone**
- All’interno del package `drone` si trova la logica di un singolo drone (classe **Drone**). 
- Attualmente si occupa delle seguenti operazioni:
  - Ingresso in modalità offboard
  - Armamento del drone
  - Decollo (**takeoff**)
  - Flocking

### **base_station**
- Contiene la logica della **BaseStation**, che coordina il flocking mandando i vettori di coesione, separazione e allineamento ai droni.
- I droni comunicano la loro posizione attraverso messaggi **GeoPing**.
- La classe **BaseStation** è un nodo ROS2 indipendente e deve essere lanciata autonomamente come i nodi dei droni.

### **obstacle_avoidance**
- Contiene una classe utilizzata da ogni istanza della classe **Drone**.
- La classe **Drone** invoca il metodo `tick()` quando controlla la presenza di ostacoli.
- Il modulo di avoidance restituisce un vettore da seguire per evitare eventuali ostacoli.

### **target_detection**
- Contiene la logica per la **detection dei target**.
- Attualmente utilizza la **Hough Transform** per rilevare sfere rosse individuate dalla telecamera.
- Il modulo ritorna il numero di target trovati alla posizione attuale e viene invocato tramite il metodo `tick()`, consentendo alla classe **Drone** di decidere quando rilevare i target.

### **parameters**
- Contiene la classe **Parameters**, caricata all'avvio di un nodo **Drone** o **BaseStation**.
- I parametri vengono letti da un file YAML situato in `/home/fourdds/.swarm/options.yaml`.
- I parametri riguardano vari aspetti della simulazione, come:
  - Velocità massima dei droni
  - Raggio massimo per l'obstacle avoidance

### **fdds_messages**
- Questo package non contiene codice, ma solo le definizioni dei messaggi **GeoPing** e **Flocking**, utilizzati dai droni e dalla base station per il coordinamento durante il flocking.
- Le definizioni dei messaggi si trovano nella cartella `msg`.

### **analysis**
- In questo package si trova lo scheletro di un programma Python (**PathView.py**) utile per fare il parsing dei file **sqlite3** generati con **ros2 bag**.
- **ros2 bag** registra i dati di determinati topic durante la simulazione, consentendo post-processing successivo.
- Attualmente, **PathView.py** mostra le traiettorie di 5 droni durante una simulazione di flocking, evidenziando con delle X le posizioni GPS in cui i droni rilevano dei target.

**Compilazione dopo le modifiche:**

- **Ricompilare i package modificati:**

  ```bash
  cd ros2_ws
  source setup.bash
  colcon build --packages-select <package_modificato>
  ```

## Gazebo
### Droni casuali
#### **Modifica dello Script di Lancio per Droni Casuali [Done]**
- **Obiettivo**: Aggiungere il lancio di droni in posizioni casuali attorno all'origine della simulazione.
- **Cosa fare**: Modifica il file di lancio della simulazione, `sitl_multiple_run.sh`, aggiungendo codice per generare posizioni casuali.
  - **Percorso del file**: `/home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh`.
  - **Modifica suggerita**: Aggiungi una funzione per generare coordinate casuali e passa queste coordinate ai droni durante il lancio.
  
### Guida per Estendere Gazebo con Sensori Dinamici
Questa guida ti aiuterà a estendere le funzionalità di Gazebo per gestire dinamicamente i sensori sui droni. Le modifiche descritte sono basate su esperienze reali e generalizzate per poter essere replicate su progetti simili. 
Clicca [qui](gazebo/sensori_dinamici.md) per approfondire.

### Guida per creare nuovi Worlds in Gazebo
Questa guida fornisce le istruzioni per creare e personalizzare i "worlds" in Gazebo, un simulatore 3D utilizzato nella piattaforma 4DDS. 
Clicca [qui](gazebo/new_world.md) per approfondire.

## Problemi comuni

- **Se i sensori non vengono caricati correttamente:**

  - **Verificare il sourcing corretto (`source setup.bash`).**
  - **Controllare il codice dei plugin per errori.**

---
