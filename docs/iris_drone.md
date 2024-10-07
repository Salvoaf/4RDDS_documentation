# Drone Swarm Coordination System with Iris Drones

## Descrizione del Progetto

Questo progetto consiste in un sistema di coordinamento per uno **sciame di droni** utilizzando **ROS 2**. Include una **stazione base** che coordina e gestisce più droni, comunicando con ciascuno per determinare il comportamento dello sciame (coesione, separazione, allineamento) e con il sistema di controllo PX4 dei droni per implementare i comandi fisici. Ogni drone riceve comandi dalla stazione base e li applica tramite il sistema di controllo.

### Componenti Principali
- **BaseStation**: La stazione base che coordina lo sciame di droni.
- **Drone**: Nodo di ogni drone nello sciame che gestisce la posizione, velocità e risponde ai comandi ricevuti dalla stazione base.
- **PX4**: Sistema di controllo hardware per il volo del drone.

## Comunicazioni del Sistema

Il sistema si basa sulle seguenti comunicazioni tra i componenti:

### Stazione Base (`BaseStation`)
- **Subscriber**: Riceve i dati dai droni tramite il topic `/swarm/geopings`.
- **Publisher**: Pubblica i dati di coesione, allineamento e separazione su `/px4_X/flocking` (dove `X` è l'ID del drone).

### Drone (`Drone`)
- **Subscriber**: Riceve dati di comportamento dalla stazione base e informazioni di posizione da PX4 tramite i topic dedicati.
- **Publisher**: Invia le proprie informazioni di posizione su `/swarm/geopings` e pubblica comandi verso PX4 (es. armamento, traiettoria).

### PX4
- **Controllo Hardware**: Esegue i comandi inviati dai droni, inclusi i comandi di armamento e di traiettoria.

## Flusso di Comunicazione
1. **Drone -> BaseStation**: I droni inviano la loro posizione tramite il topic `/swarm/geopings`.
2. **BaseStation -> Drone**: La stazione base elabora la posizione di tutti i droni e pubblica i dati comportamentali (`cohesion`, `alignment`, `separation`) su `/px4_X/flocking` per ogni drone.
3. **Drone -> PX4**: Ogni drone invia comandi al sistema PX4 per implementare i comandi di volo fisico, come l'armamento e la traiettoria.
4. **PX4 -> Drone**: PX4 fornisce aggiornamenti sulla posizione del drone.

## Requisiti di Sistema
- **ROS 2 Foxy o successivo**
- **PX4 Autopilot**
- **Eigen Library** (per la gestione delle matrici e delle operazioni matematiche)
- **GCC** o **Clang** per la compilazione del codice

## Come Compilare ed Eseguire
### Compilazione
1. Clona il repository:
   ```sh
   git clone https://github.com/tuo_utente/drone_swarm.git
   cd drone_swarm
   ```
2. Compila il progetto utilizzando `colcon`:
   ```sh
   colcon build
   ```

### Esecuzione
1. Inizializza ROS 2:
   ```sh
   source /opt/ros/foxy/setup.bash
   source install/setup.bash
   ```
2. Esegui la stazione base:
   ```sh
   ros2 run base_station BaseStation <numero_droni>
   ```
3. Esegui i droni:
   ```sh
   ros2 run drone Drone <numero_droni>
   ```

## Struttura del Codice
- **`base_station/`**: Contiene il codice per la gestione della stazione base.
- **`drone/`**: Contiene il codice per i singoli droni dello sciame.
- **`parameters/`**: Definisce i parametri utilizzati dai nodi, inclusi i fattori di coesione, allineamento e separazione.

## Logica di Coordinamento dello Sciame
- **GeoPing**: I droni inviano periodicamente la loro posizione tramite messaggi di tipo `GeoPing` alla stazione base.
- **Comportamento di Sciame**: La stazione base calcola il centro di massa dei vicini, la direzione di allineamento e la forza di separazione, pubblicando questi dati per ogni drone.
- **Modalità Offboard e Armamento**: Ogni drone richiede al sistema PX4 di entrare in modalità offboard e di armarsi prima di eseguire i comandi dello sciame.

## Contributori
- **Nome del Contributore**: Ruolo
- **Salvo**: Sviluppatore principale e coordinatore del progetto.

## Note e Limitazioni
- Il sistema attuale funziona in **modalità simulazione**. È necessario il supporto di un simulatore come **Gazebo** per testare il sistema.
- La logica di evitamento ostacoli ha precedenza su coesione e allineamento per garantire la sicurezza del volo.

## Licenza
Questo progetto è distribuito sotto la licenza MIT. Vedi il file `LICENSE` per maggiori dettagli.
