# Tutorial per l'utente e lo sviluppatore della piattaforma 4DDS

---

## a) Tutorial per l'utente

### Installazione di WSL2 su Windows 11

Per utilizzare la piattaforma 4DDS, è necessario installare il Sottosistema Windows per Linux (WSL2), che consente di eseguire un ambiente Linux all'interno di Windows. Attualmente, l'ambiente Linux utilizzato è Ubuntu 22.04.

**Passaggi per l'installazione:**

1. **Aprire PowerShell come amministratore.**
2. **Digitare il comando seguente:**

   ```powershell
   wsl --install
   ```

3. **Seguire le istruzioni sullo schermo.**

Se avete dubbi o problemi durante l'installazione, consultate la [documentazione ufficiale di Microsoft](https://docs.microsoft.com/it-it/windows/wsl/install).

### Importazione del disco virtuale 4DDS in WSL2

Il disco virtuale 4DDS contiene tutti gli strumenti necessari per simulare uno sciame di droni:

- **Gazebo Classic:** Simulatore 3D con l'ambiente di esplorazione, il modello fisico del drone e i target.
- **PX4:** Autopilota che gestisce il controllo a basso livello del drone.
- **ROS2:** Middleware per la comunicazione e il coordinamento dei droni.
- **XRCE-DDS:** Software che permette la comunicazione tra PX4 e ROS2.

**Procedura di importazione:**

1. **Scaricare il file `4DDS.zip`.**
2. **Estrarre il file `4DDS.tar` utilizzando un programma come 7zip.**
3. **Aprire PowerShell e navigare nella cartella contenente `4DDS.tar`.**
4. **Eseguire il comando:**

   ```powershell
   wsl --import 4DDS . 4DDS.tar
   ```

   Questo creerà un disco virtuale chiamato `ext4.vhdx` nella cartella corrente.

5. **Eliminare `4DDS.tar` per risparmiare spazio, se lo desiderate.**

### Accesso all'ambiente 4DDS

Per accedere all'ambiente Linux di 4DDS:

1. **Aprire PowerShell.**
2. **Digitare:**

   ```powershell
   4DDS
   ```

3. **Cambiare utente a `fourdds`:**

   ```bash
   su fourdds
   cd
   ```

   *Nota:* La password per l'utente `fourdds` è `4DDS1234`.

### Installazione di Visual Studio Code

Per modificare e gestire i file all'interno dell'ambiente 4DDS, è consigliato utilizzare Visual Studio Code con l'estensione per WSL.

**Passaggi:**

1. **Scaricare e installare [Visual Studio Code](https://code.visualstudio.com/).**
2. **Seguire la [guida all'uso di Visual Studio Code con WSL](https://code.visualstudio.com/docs/remote/wsl).**
3. **Impostare `/home/fourdds` come cartella di lavoro.**

### Avvio di una simulazione di missione e acquisizione dei log

Per lanciare una simulazione, aprire quattro schede (tab) nel terminale.

#### **Tab 1: Avvio del broker XRCE-DDS**

```bash
MicroXRCEAgent udp4 –p 8888
```

Questo processo media la comunicazione tra PX4 e ROS2.

#### **Tab 2: Avvio della simulazione con Gazebo**

```bash
source ws/setup.bash
./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n <numero_droni> -w find_targets
```

**Note:**

- **Per chiudere Gazebo, premere `Ctrl-C` nella scheda in cui è stato lanciato.**
- **Se rimangono processi attivi, eseguire:**

  ```bash
  killall -9 gzclient
  killall -9 gzserver
  ```

Si aprirà una finestra con l'ambiente di simulazione (Fig.1), dove è possibile visualizzare e interagire con droni, ostacoli e target.

![Fig.1 – Ambiente di simulazione Gazebo Classic](images/simulazione_GazeboClassic.png)

#### **Tab 3: Avvio della logica dei droni con ROS2**

```bash
cd ros2_ws
source setup.bash
bash run.sh -n <numero_droni>
```

Questo comando avvia la logica dei droni, permettendo loro di decollare e volare in formazione con evitamento degli ostacoli.

#### **Tab 4: Registrazione e conversione dei log**

**Per registrare i log:**

```bash
cd ros2_ws
source setup.bash
bash register.bash
```

- **Premere `Ctrl-C` per interrompere la registrazione.**
- **Verrà creata una cartella `log_<timestamp>` contenente un file `.db3`.**

**Per convertire i log in CSV:**

```bash
ros2 run analysis ConvertToCSV log_<timestamp>/log_<timestamp>.db3
```

- **Genererà i file `geopings.csv` e `target_position.csv`.**
- **Spostarli in una cartella dedicata per analisi successive.**

### Modifica dei parametri di simulazione

Per modificare i parametri della simulazione:

- **Modificare il file `/home/fourdds/.swarm/options.yaml`.**

Se apportate modifiche ai package ROS2 (ad esempio, `obstacle_avoidance`, `target_detection`, `drone`, `base_station`), è necessario ricompilare il progetto:

```bash
cd ros2_ws
source setup.bash
colcon build --packages-select target_detection obstacle_avoidance base_station drone
```

### Consigli per le simulazioni

- **Se i droni diventano lenti ad armarsi, riavviare tutti i processi.**
- **Non è necessario eseguire nuovamente `source setup.bash` a meno che non chiudiate le schede del terminale.**
- **Per creare scenari personalizzati:**

  1. **Avviare Gazebo con il comando `gazebo`.**
  2. **Personalizzare l'ambiente.**
  3. **Salvare il mondo nella cartella:**

     ```
     ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo_classic/worlds/
     ```

### Architettura software della simulazione

Nella nuova architettura:

- **ROS2** facilita la comunicazione tra nodi e autopiloti PX4.
- **I nodi ROS2** possono pubblicare e sottoscrivere messaggi su specifici topic.
- **Il broker XRCE-DDS** gestisce lo smistamento dei messaggi.

**Esempio di comunicazione:**

- **PX4** pubblica dati GPS su `/px4_1/fmu/out/vehicle_gps_position`.
- **Un nodo ROS2** può sottoscrivere questo topic per ricevere i dati.

---

## b) Tutorial per lo sviluppatore

### Modifica e personalizzazione della piattaforma

L'ambiente 4DDS include tutto il necessario per simulare sciami di droni:

- **PX4:** Autopilota per il controllo a basso livello.
- **ROS2:** Middleware per la logica di alto livello.
- **Gazebo Classic:** Simulatore 3D integrato con PX4.
- **Gazebo-ROS-Pkgs:** Plugin per simulare sensori aggiuntivi.

### Modifiche a PX4

**Aggiunta di sensori ai modelli:**

- **Iris quadcopter:**
  - Aggiunto LIDAR a 360 gradi.
  - Aggiunta camera rivolta verso il basso.
- **Rover:**
  - Ridimensionato per adattarsi alle esigenze di simulazione.
- **BlueROV2:**
  - Aggiunti sensori IMU.
  - Creato template per spawn multi-drone.

**Modifiche ai moduli di controllo:**

- **Rover:**
  - Accetta setpoint di velocità in coordinate NED.
- **BlueROV2:**
  - Riscritti moduli di controllo per velocità e profondità.

**Attenzione:** Non eseguire `make distclean` nella cartella `PX4-Autopilot`, poiché cancellerà tutte le modifiche.

### Modifiche a ROS2

**Struttura dei package:**

- **drone:** Logica del singolo drone (armamento, decollo, flocking).
- **base_station:** Coordina lo sciame e gestisce il flocking.
- **obstacle_avoidance:** Gestisce l'evitamento degli ostacoli.
- **target_detection:** Rileva i target utilizzando tecniche come la trasformata di Hough.
- **parameters:** Gestisce i parametri di simulazione da un file YAML.
- **fdds_messages:** Definisce i messaggi utilizzati per la comunicazione.
- **analysis:** Contiene strumenti per l'analisi dei dati (es. `PathView.py`).

**Compilazione dopo le modifiche:**

- **Ricompilare i package modificati:**

  ```bash
  cd ros2_ws
  source setup.bash
  colcon build --packages-select <package_modificato>
  ```

### Modifiche a Gazebo

**Script di lancio:**

- **`sitl_multiple_run.sh`:** Modificato per lo spawn casuale dei droni.

**Plugin per sensori:**

- **Modifiche ai plugin LIDAR e camera per supportare più sensori sullo stesso veicolo.**
- **I plugin si trovano in `/home/fourdds/ws/gazebo_plugins/src`.**

**Per aggiungere nuovi sensori:**

1. **Individuare il plugin necessario.**
2. **Apportare le modifiche per gestire più istanze del sensore.**
3. **Ricompilare i plugin:**

   ```bash
   cd ws
   colcon build --symlink-install
   source setup.bash
   ```

**Risoluzione dei problemi:**

- **Se i sensori non vengono caricati correttamente:**

  - **Verificare il sourcing corretto (`source setup.bash`).**
  - **Controllare il codice dei plugin per errori.**

---

## Riferimenti

- [PX4 Autopilot su GitHub](https://github.com/PX4/PX4-Autopilot)
- [Documentazione PX4 su ROS2](https://docs.px4.io/main/en/ros/ros2)
- [Simulazione con Gazebo e PX4](https://docs.px4.io/v1.12/en/simulation/gazebo.html)
- [Tutorial ROS2 in C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Gazebo Classic Tutorials](https://classic.gazebosim.org/tutorials)
- [uuv_simulator su GitHub](https://github.com/uuvsimulator/uuv_simulator)
- [Guida all'uso di Visual Studio Code con WSL](https://code.visualstudio.com/docs/remote/wsl)
