
# Tutorial per l'Utente e lo Sviluppatore della Piattaforma 4DDS

## 1. Tutorial per l'utente

### Come si installa VSL su Windows 11
1. Installare il sottosistema Linux per Windows WSL2:
   ```bash
   wsl --install
   ```
   Segui le istruzioni su [questa pagina](https://aka.ms/wsl2) per eventuali dettagli aggiuntivi.

### Come si caricano i container 4DDS
1. Scarica il file `4DDS.tar` dal pacchetto `4DDS.zip`.
2. Estrarre il file `.tar` utilizzando un software di decompressione (es. 7zip).
3. Importare il disco virtuale usando:
   ```bash
   wsl --import 4DDS . 4DDS.tar
   ```

### Come si crea uno scenario e si fa partire
1. Per eseguire uno scenario di simulazione:
   ```bash
   MicroXRCEAgent udp4 –p 8888
   ```

   Poi, nella seconda tab della console:
   ```bash
   source ws/setup.bash
   ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n <numero_droni> -w find_targets
   ```

2. Per chiudere Gazebo:
   ```bash
   killall –9 gzclient
   killall –9 gzserver
   ```

### Metriche online
1. Contatore dei target trovati visibile direttamente nella missione.

### Metriche offline
1. Tabella delle posizioni dei droni (timestamp, ID, posizione, orientamento, velocità).
2. Tabella dei target trovati.
3. Tabella delle collisioni tra droni.
4. Script per la conversione dei log in CSV.

## 2. Tutorial per lo sviluppatore
### Come si modificano le logiche del sistema
1. Eseguire modifiche su ROS2, PX4 e le logiche di swarm nei file in `/ros2_ws/src`.
2. Ricompilare i package modificati:
   ```bash
   source setup.bash
   colcon build --packages-select target_detection obstacle_avoidance base_station drone
   ```

### Come lanciare simulazioni con scenari personalizzati
1. Esegui il comando `gazebo` per avviare l'editor di scenari.
2. Personalizza i mondi di simulazione in `/PX4-Autopilot/Tools/simulation/gazebo-classic/worlds/`.

## 3. Architettura della simulazione
1. **PX4**: gestione di basso livello.
2. **ROS2**: gestione di alto livello del coordinamento e dell'elaborazione dei dati dai sensori.
3. **Gazebo Classic**: simulatore 3D per la fisica dei droni.
4. **XRCE-DDS**: gestione della comunicazione tra i componenti.
