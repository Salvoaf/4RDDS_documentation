# Comunicazioni tra Stazione Base e Droni nello Sciame

Di seguito vengono riportate le tabelle che descrivono le comunicazioni tra i diversi componenti del sistema ROS 2, includendo i topic utilizzati, i publisher, i subscriber, e i client/servizi. Questo aiuta a comprendere come i droni e la stazione base interagiscono.

## Tabella 1: Comunicazioni della Stazione Base (`BaseStation`)
| Componente        | Tipo        | Topic/Client                                   | Pubblicato da   | Ricevuto da     | Descrizione                                                                                   |
|-------------------|-------------|-----------------------------------------------|-----------------|-----------------|-----------------------------------------------------------------------------------------------|
| Subscriber        | Topic       | `/swarm/geopings`                             | Drone           | BaseStation     | La stazione base riceve la posizione e la velocità dai droni.                                 |
| Publisher         | Topic       | `/px4_X/flocking` (dove X è l'ID del drone)    | BaseStation     | Drone           | Pubblica i dati di **cohesione, allineamento, separazione** per ciascun drone.                |
| Callback          | Funzione    | `geoPingCallback()`                           | BaseStation     | -               | Gestisce l'aggiornamento delle posizioni dei droni e pubblica il comportamento di sciame.     |

## Tabella 2: Comunicazioni del Drone (`Drone`)
| Componente        | Tipo        | Topic/Client                                  | Pubblicato da   | Ricevuto da     | Descrizione                                                                                   |
|-------------------|-------------|----------------------------------------------|-----------------|-----------------|-----------------------------------------------------------------------------------------------|
| Subscriber        | Topic       | `<vehicleNamespace>/fmu/out/vehicle_gps_position` | PX4          | Drone           | Riceve i dati GPS dal sistema PX4 per aggiornare la posizione del drone.                      |
| Subscriber        | Topic       | `<vehicleNamespace>/fmu/out/vehicle_local_position` | PX4        | Drone           | Riceve la posizione locale del drone.                                                         |
| Subscriber        | Topic       | `<vehicleNamespace>/flocking`                | BaseStation     | Drone           | Riceve dati sul comportamento di sciame dalla stazione base.                                  |
| Publisher         | Topic       | `<vehicleNamespace>/fmu/in/vehicle_command`  | Drone           | PX4             | Pubblica i comandi per il drone (es. armamento, takeoff).                                     |
| Publisher         | Topic       | `<vehicleNamespace>/fmu/in/trajectory_setpoint` | Drone         | PX4             | Pubblica i setpoints di traiettoria del drone.                                                |
| Publisher         | Topic       | `/swarm/geopings`                            | Drone           | BaseStation     | Invia periodicamente la posizione e la velocità del drone alla stazione base.                 |
| Publisher         | Topic       | `/target_position`                           | Drone           | -               | Pubblica la posizione target del drone (per altri componenti del sistema).                    |
| Client            | Servizio    | `<vehicleNamespace>/fmu/vehicle_command`     | Drone           | PX4             | Richiede servizi specifici (es. armamento, passaggio alla modalità offboard).                 |

## Tabella 3: Comunicazioni Generali tra Stazione Base e Droni
| Componente           | Direzione della Comunicazione | Tipo di Comunicazione  | Pubblicato da   | Ricevuto da     | Descrizione                                                                                               |
|----------------------|------------------------------|------------------------|-----------------|-----------------|-----------------------------------------------------------------------------------------------------------|
| **Drone -> BaseStation** | Droni inviano messaggi    | Publisher (`GeoPing`)   | Drone           | BaseStation     | Ogni drone invia la propria posizione e velocità utilizzando il topic `/swarm/geopings`.                  |
| **BaseStation -> Drone** | Stazione Base invia messaggi | Publisher (`Flocking`) | BaseStation     | Drone           | La stazione base pubblica i dati di comportamento (coesione, allineamento, separazione) ai droni.        |
| **Drone -> PX4**         | Droni controllano PX4     | Publisher (`VehicleCommand`, `TrajectorySetpoint`) | Drone | PX4           | Ogni drone pubblica comandi di controllo verso il sistema PX4 per eseguire operazioni come armamento e movimenti. |
| **PX4 -> Drone**         | PX4 invia dati ai droni   | Subscriber (`SensorGps`, `VehicleLocalPosition`) | PX4   | Drone           | Ogni drone riceve aggiornamenti sulla propria posizione globale e locale tramite topic specifici di output da PX4. |

## Riassunto delle Comunicazioni
- **Topic**:
  - La **stazione base** riceve le posizioni dai droni tramite `/swarm/geopings` e pubblica i dati di comportamento sui topic dedicati a ciascun drone (`/px4_X/flocking`).
  - I **droni** comunicano sia con la stazione base sia con il sistema **PX4**. In particolare:
    - I droni pubblicano i propri dati di posizione sul topic `/swarm/geopings` per la stazione base.
    - La stazione base restituisce dati comportamentali sui topic dedicati di ciascun drone.
    - Ogni drone comunica direttamente con il sistema **PX4** per il controllo fisico (comandi di volo e setpoints).

- **Client e Servizi**:
  - I droni utilizzano un client per inviare comandi specifici a PX4, come l'**armamento** e l'**attivazione della modalità offboard**.

## Come Immaginare il Sistema
- La **Stazione Base** è il nodo centrale che coordina lo sciame di droni. Riceve costantemente i dati di posizione dai droni, elabora il comportamento collettivo (coesione, separazione e allineamento) e invia questi comandi a ciascun drone. Questo consente a ogni drone di navigare in un ambiente evitando ostacoli e mantenendo una posizione relativa rispetto agli altri droni.
- Ogni **Drone** è responsabile di:
  - Comunicare la propria posizione alla stazione base.
  - Eseguire i comandi comportamentali ricevuti dalla stazione base per navigare come parte dello sciame.
  - Gestire la comunicazione con il sistema **PX4** per implementare comandi di volo fisico come armamento e impostazione della traiettoria.
- **PX4** è il sistema che controlla l'hardware del drone e garantisce che i comandi inviati dai nodi ROS vengano effettivamente eseguiti.

Le tabelle e il riassunto forniscono una visione chiara delle comunicazioni tra le parti coinvolte, facilitando la comprensione del sistema completo.
