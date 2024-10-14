# **Quadricottero Iris**

Tipo: Aria

- **Percorso file**: `/home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl-gazebo_classic/models/iris/iris.sdf.jinja`

## **Sensori**
1. **IMU (Inertial Measurement Unit)**: 
   - Misura l'accelerazione e la rotazione del drone.
   - Configurato tramite il plugin `libgazebo_imu_plugin.so`.

2. **LiDAR**: 
   - Scanner laser per la mappatura e rilevamento ostacoli.
   - Definito tramite il modello `4dds_rplidar`.

3. **Fotocamera FPV (First-Person View)**: 
   - Utilizzata per la visione in prima persona.
   - Definita tramite il modello `fpv_cam`.

4. **GPS**: 
   - Utilizzato per ottenere la posizione locale del drone.
   - Definito tramite il modello GPS incluso.

5. **Magnetometro**: 
   - Misura il campo magnetico per determinare l'orientamento rispetto al campo magnetico terrestre.
   - Configurato tramite il plugin `libgazebo_magnetometer_plugin.so`.

6. **Barometro**: 
   - Misura la pressione atmosferica per stimare l'altitudine.
   - Configurato tramite il plugin `libgazebo_barometer_plugin.so`.

## **Attuatori**
1. **Rotor_0**: 
   - Uno dei quattro rotori principali per il volo.
   - Controllato tramite giunto `revolute` e il plugin `libgazebo_motor_model.so`.

2. **Rotor_1**: 
   - Simile a Rotor_0, controllato da un giunto `revolute` e plugin.

3. **Rotor_2**: 
   - Simile a Rotor_0, ma con direzione di rotazione opposta (CW).

4. **Rotor_3**: 
   - Simile a Rotor_2, controllato da giunto e plugin.

Ogni rotore ha un controllo di velocità e direzione tramite i relativi plugin.

---