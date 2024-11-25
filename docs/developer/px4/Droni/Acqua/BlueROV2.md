# **BlueROV2**
- **Percorso file**: `/home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/uuv_bluerov2_heavy/uuv_bluerov2_heavy.sdf.jinja`

## **Sensori**
1. **IMU (Inertial Measurement Unit)**: 
   - Misura l'accelerazione e la rotazione del drone.
   - Configurato tramite il plugin `gazebo_imu_plugin`.
   - 
2. **LiDAR**: 
   - Scanner laser per la mappatura e rilevamento ostacoli.
   - Definito tramite il modello `4dds_rplidar`.

3. **Fotocamera FPV (First-Person View)**: 
   - Utilizzata per la visione in prima persona.
   - Definita tramite il modello `fpv_cam`.

4. **GPS**: 
   - Utilizzato per ottenere la posizione locale.
   - Definito tramite l'inclusione del modello GPS.

5. **Magnetometro**: 
   - Misura il campo magnetico per determinare l'orientamento rispetto al campo magnetico terrestre.
   - Configurato tramite il plugin `magnetometer_plugin`.

6. **Barometro**: 
   - Misura la pressione atmosferica per stimare l'altitudine.
   - Configurato tramite il plugin `barometer_plugin`.

## **Attuatori**
1. **Thruster1**: 
   - Propulsore per generare movimento.
   - Controllato tramite giunto `revolute` e il plugin `libgazebo_motor_model.so`.

2. **Thruster2**: 
   - Stesso setup di Thruster1 con controllo del movimento.

3. **Thruster3**: 
   - Simile agli altri thruster, controllato da un giunto `revolute` e plugin.

4. **Thruster4**: 
   - Anche questo è un propulsore controllato da giunto e plugin.

5. **Thruster5**: 
   - Propulsore con giunto `revolute` e plugin di controllo motore.

6. **Thruster6**: 
   - Stesso setup con giunto e controllo del motore.

7. **Thruster7**: 
   - Controllato da un giunto `revolute` e plugin per il controllo della velocità.

8. **Thruster8**: 
   - Ultimo thruster con configurazione simile agli altri, controllato da giunto e plugin.


- **Moduli di controllo**: Modifiche per accettare setpoint di velocità in coordinate NED.

**Importante**: Non utilizzare `make distclean` in PX4-Autopilot, altrimenti tutte le modifiche andranno perse.
