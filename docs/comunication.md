### Diagramma di interazione del drone 
path = `/home/fourdds/ros2_ws/src/drone/src/Drone.cpp`

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
