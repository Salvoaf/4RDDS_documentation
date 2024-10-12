# Guida per Estendere Gazebo con Sensori Dinamici

Questa guida ti aiuterà a estendere le funzionalità di Gazebo per gestire dinamicamente i sensori sui droni. Le modifiche descritte sono basate su esperienze reali e generalizzate per poter essere replicate su progetti simili.


## **Modifica del Plugin per Gestire Sensori Dinamici - Far pubblicare topic ai sensori [Migliorare]**
- **Obiettivo**: Consentire il supporto di sensori dinamici, come LIDAR e telecamere, pubblicando su topic differenti.
- **Cosa fare**:
  - Individua i file plugin relativi ai sensori, in genere sotto la directory `/ws/gazebo_plugins/src`.
  - Aggiungi la logica per distinguere tra i vari sensori in base ai topic pubblicati.
  - **Esempio**: Modifica i plugin dei sensori in modo che ciascun LIDAR, o sensore simile, possa pubblicare su un topic univoco per identificare correttamente il sensore all'interno di un drone.

## **Gestione di Più Sensori per Veicolo [Migliorare con esempio] **
- **Obiettivo**: Aggiungere il supporto per più sensori dello stesso tipo su ciascun veicolo.
- **Cosa fare**:
  - Modifica il plugin del sensore, ad esempio `gazebo_ros_ray_sensor.cpp` per LIDAR.
  - Nella funzione `::Load()`, aggiungi un controllo per la presenza di un tag speciale, come `<px4/>`, per identificare i sensori e incrementare un ID per distinguere tra i vari sensori dello stesso tipo.
  - **Esempio di implementazione con LIDAR**:
    - Il **primo LIDAR** può pubblicare su `px4_X/rplidar` per l'obstacle avoidance.
    - Il **secondo LIDAR** può pubblicare su `px4_X/downlidar` per monitorare il terreno.

## **Estensione per Nuovi Sensori [Migliorare con esempio]**
- **Obiettivo**: Aggiungere nuovi sensori al drone.
- **Cosa fare**:
  - Trova il plugin adatto al sensore che vuoi aggiungere.
  - Segui lo stesso approccio del plugin del LIDAR per gestire i topic pubblicati da ciascun sensore.

## **Compilazione delle Modifiche[Done]**
- Dopo aver apportato modifiche ai plugin, è necessario ricompilare i plugin di Gazebo:
  ```bash
  colcon build --symlink-install
  ```
- Fai il sourcing dei pacchetti appena compilati:
  ```bash
  source setup.bash
  ```

## **Risoluzione dei Problemi di Spawn**
- **Problema comune**: I sensori potrebbero non caricarsi correttamente durante lo spawn.
- **Soluzione**:
  1. Assicurati di aver fatto correttamente il sourcing dei **gazebo-ros-pkgs**.
  2. Verifica che non ci siano errori nel codice relativo allo spawn dinamico dei sensori.
.