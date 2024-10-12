## Tutorial per l'utente

### Come installare WSL su Windows 11

1. Apri **PowerShell** come amministratore e digita il seguente comando per installare WSL2:

   ```
   wsl --install
   ```

2. Per ulteriori dettagli, consulta la [pagina ufficiale di WSL](https://docs.microsoft.com/windows/wsl/install).

### Come caricare i container 4DDS

1. Dopo aver installato WSL2, importa il disco virtuale 4DDS seguendo questi passaggi:

   - Estrai il file `4DDS.zip` utilizzando un programma come **7zip** per ottenere il file `4DDS.tar` (circa 19-20GB).
   - Apri **Windows PowerShell**, vai nella cartella contenente il file `4DDS.tar` e digita:
     ```
     wsl --import 4DDS . 4DDS.tar
     ```

2. Al termine dell'operazione, sarà presente un disco virtuale chiamato `ext4.vhdx` nella cartella attuale. Puoi eliminare `4DDS.tar` per risparmiare spazio, ma sarà possibile recuperarlo nuovamente da `4DDS.zip`.

### Come creare uno scenario e farlo partire

1. Apri **PowerShell** e digita il comando:

   ```
   wsl -d 4DDS
   ```

2. Accedi come utente `fourdds`:

   ```
   su fourdds
   cd
   ```

   - Password dell'utente: `4DDS1234`

3. Installa **Visual Studio Code** seguendo le istruzioni al [link ufficiale](https://code.visualstudio.com/docs/remote/wsl).

   - Usa `/home/fourdds` come workspace folder di VS Code.

### Metriche di simulazione

- **Metriche online**:

  - Contatore dei target trovati durante una missione di 10 minuti.
  - Clock di sistema (300 ms) e tick di logging (10 sec).

- **Metriche offline**:

  1. Tabella delle posizioni dei droni (timestamp, ID drone, posizione, orientamento, velocità).
  2. Tabella dei target trovati (timestamp, ID drone, posizione, orientamento, velocità).
  3. Tabella delle collisioni (timestamp, ID droni, posizioni, orientamenti, velocità).

### Script di log

- **Conversione log in CSV**: Script per convertire i log in formato CSV (per analisi in Excel).
- **Processing log**: Script per calcolare la distanza media tra i droni.

### Lanciare una simulazione di missione e ottenere i log degli eventi

1. Apri **PowerShell** e prepara **quattro tab** sulla home dell'utente `fourdds`:
   - **Tab 1**: Avvia il processo di comunicazione tra PX4 e ROS2 con il comando:

     ```
     MicroXRCEAgent udp4 -p 8888
     ```

   - **Tab 2**: Attiva la simulazione:

     ```
     cd ws
     source setup.bash
     cd ..
     ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n <numero_droni> -w find_targets
     ```

     - **Note:**

      - **Per chiudere Gazebo, premere `Ctrl-C` nella scheda in cui è stato lanciato.**
      - **Se rimangono processi attivi, eseguire:**

        ```bash
        killall -9 gzclient
        killall -9 gzserver
        ```

      Si aprirà una finestra con l'ambiente di simulazione (Fig.1), dove è possibile visualizzare e interagire con droni, ostacoli e target.

      ![Fig.1 – Ambiente di simulazione Gazebo Classic](images/simulazione_GazeboClassic.png)


   - **Tab 3**: Attiva la logica dei droni (flocking e obstacle avoidance):

     ```
     cd ros2_ws
     source setup.bash
     bash run.sh -n <numero_droni>
     ```

   - **Tab 4**: Registra i log e converti in CSV:

     ```
     cd ros2_ws
     source setup.bash
     bash register.bash
     ```

     - Dopo la registrazione dei log, esegui:
       ```
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

### Consigli per il lancio di simulazioni

- Dopo aver modificato la logica di ROS2, potrebbe non essere necessario chiudere e riaprire Gazebo e il broker XRCE-DDS. Tuttavia, se i droni diventano lenti ad armarsi, conviene riavviare tutto da capo.

- Per creare scenari personalizzati (.world), apri Gazebo con il comando `gazebo` e salva il mondo nella cartella `~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo_classic/worlds/`.

### Architettura software della simulazione

Nella nuova architettura:

- **ROS2** facilita la comunicazione tra nodi e autopiloti PX4.
- **I nodi ROS2** possono pubblicare e sottoscrivere messaggi su specifici topic.
- **Il broker XRCE-DDS** gestisce lo smistamento dei messaggi.

**Esempio di comunicazione:**

- **PX4** pubblica dati GPS su `/px4_1/fmu/out/vehicle_gps_position`.
- **Un nodo ROS2** può sottoscrivere questo topic per ricevere i dati.

---


### Diagramma di interazione del drone 

Il diagramma di interazione del drone rappresenta due cicli principali:

**legenda:**
```
-topic @ stato attuale : funziona da dove parte la ricezione o l'invio del topic
-topic @ stato attuale -> stato successibo : funziona da dove parte la ricezione o l'invio del topic
```

1. **Ciclo di Bootstrap**: Questo ciclo si ripete ogni 'options.controllerTimerMs' millisecondi e parte chiamando la funzione `bootstrap()`. Il suo scopo è gestire il controllo offboard, l'armamento e il movimento del drone.


```plantuml
@startuml
Px4 <- Drone : /fmu/in/offboard_control_mode @ NULL : heartbeat() # avvisa della sua esistenza
Px4 <- Drone : /fmu/in/trajectory_setpoint @ NULL : sendGoto() # richiede una certa altezza
Px4 <- Drone : /fmu/in/vehicle_command @ init -> OffboardRequested : requestOffboardControl()
Px4 --> Drone: serviceDone == True @ OffboardRequested -> WaitForStableOffboard : commandResponseCallback()
Px4 <- Drone : /fmu/in/vehicle_command @ WaitForStableOffboard -> ArmRequested : requestArm()
Px4 --> Drone: serviceDone == True @ ArmRequested -> Armed: commandResponseCallback()
Px4 <- Drone : /fmu/in/trajectory_setpoint @ Armed : requestTakeoff() -> sendGoto()
Px4 -> Drone : /fmu/out/vehicle_load_position @ Armed: localPositionSubscriptionCallback()
Drone -> Base_Station: /swarm/geopings @ Armed : sendGeoPing()
Base_Station -> Drone : /px4_X/flocking @ Armed : flockingSubscriptionCallback()
Px4 <- Drone : /fmu/in/trajectory_setpoint @ Armed : sendVelocity()
Drone -> Base_Station: /swarm/geopings @ Armed : End sendGeoPing()
Px4 -> Drone : vehicleNamespace + "_camera/camera/image_raw/compressed" @ NULL : tick() # drone acquisisce l'immagine che viene elaborata
@enduml
```
Incolla il codice sopra [qui](https://www.plantuml.com/).

2. **Ciclo di Target Detection**: C'è un'altra iterazione del drone che si ripete ogni 750 ms. Questo ciclo parte chiamando la funzione `targetDetection()`, che avvia il processo descritto dal nuovo diagramma.

- Il modulo TargetDetection si sottoscrive al topic per ricevere le immagini.
- Quando viene pubblicata una nuova immagine, `cameraSubscriptionCallback` viene chiamato e salva l'immagine in `lastCameraData`.
- Successivamente, quando `tick()` viene chiamato, elabora l'ultima immagine disponibile (`lastCameraData`) per verificare se ci sono obiettivi rilevati.

```plantuml
@startuml
@startuml
Px4 -> Drone : vehicleNamespace + "_camera/camera/image_raw/compressed" @ NULL : tick() # Px4 invia continuamente al drone le immagini acquisite dal sensore con tick() processa l'ultima per la detection
@enduml
@enduml
```
Incolla il codice sopra [qui](https://www.plantuml.com/)
