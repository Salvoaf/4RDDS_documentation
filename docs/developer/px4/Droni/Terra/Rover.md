# **Rover**

Tipo: Terra

- **Percorso file**: `/home/fourdds/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl-gazebo_classic/models/rover/rover.sdf.jinja`

## **Sensori**
1. **IMU (Inertial Measurement Unit)**: 
   - Misura l'accelerazione e la rotazione del rover.
   - Configurato tramite il plugin `libgazebo_imu_plugin.so`.

2. **LiDAR**: 
   - Scanner laser per la mappatura e rilevamento ostacoli.
   - Definito tramite il modello `4dds_rplidar`.

3. **GPS**: 
   - Utilizzato per ottenere la posizione locale del rover.
   - Definito tramite il modello GPS incluso.

4. **Magnetometro**: 
   - Misura il campo magnetico per determinare l'orientamento rispetto al campo magnetico terrestre.
   - Configurato tramite il plugin `libgazebo_magnetometer_plugin.so`.

5. **Barometro**: 
   - Misura la pressione atmosferica per stimare l'altitudine.
   - Configurato tramite il plugin `libgazebo_barometer_plugin.so`.

## **Attuatori**
1. **Ruota anteriore sinistra**: 
   - Controllata tramite giunto `revolute`, definita come `front_left_wheel_joint`.
   - Controllo di posizione tramite `front_left_steering_joint`.

2. **Ruota anteriore destra**: 
   - Controllata tramite giunto `revolute`, definita come `front_right_wheel_joint`.
   - Controllo di posizione tramite `front_right_steering_joint`.

3. **Ruota posteriore sinistra**: 
   - Controllata tramite giunto `revolute`, definita come `rear_left_wheel_joint`.
   - Controllo di velocità tramite `rear_left_wheel_drive`.

4. **Ruota posteriore destra**: 
   - Controllata tramite giunto `revolute`, definita come `rear_right_wheel_joint`.
   - Controllo di velocità tramite `rear_right_wheel_drive`.

Ogni ruota è controllata tramite giunti con parametri specifici per il movimento e la direzione del rover.

---