
# Installazione di 4DDS e componenti

Questo documento descrive in dettaglio come installare e configurare l'ambiente necessario per utilizzare il progetto 4DDS, partendo dall'installazione di WSL su Windows 11 fino all'avvio di uno scenario con Px4, ROS2, e Gazebo.

### Come installare WSL su Windows 11

1. **Installazione di WSL2:**

   - Apri **PowerShell** come amministratore. Per farlo, cerca "PowerShell" nel menu Start, clicca con il tasto destro e seleziona **Esegui come amministratore**.

   - Digita il seguente comando per installare WSL2:

     ```
     wsl --install
     ```

   - Questo comando installerà l'ultima versione di WSL e la distribuzione Ubuntu come sistema Linux predefinito.

2. **Documentazione aggiuntiva:**

   - Per ulteriori dettagli sull'installazione e la configurazione di WSL, consulta la [pagina ufficiale di WSL](https://docs.microsoft.com/windows/wsl/install).

### Come caricare i container 4DDS

1. **Importazione del disco virtuale 4DDS:**

   - Dopo aver installato WSL2, è necessario importare il disco virtuale del progetto 4DDS. Segui questi passaggi:
     - Prima di tutto, estrai il file `4DDS.zip` utilizzando un programma come **7zip** per ottenere il file `4DDS.tar`. Questo file ha una dimensione di circa 19-20 GB.
     - Successivamente, apri **Windows PowerShell** e spostati nella cartella contenente il file `4DDS.tar` digitando:
       ```
       cd percorso/della/cartella
       ```
     - Importa il disco virtuale utilizzando il seguente comando:
       ```
       wsl --import 4DDS . 4DDS.tar
       ```

2. **Pulizia dello spazio di archiviazione:**

   - Una volta completata l'importazione, verrà creato un disco virtuale denominato `ext4.vhdx` nella cartella corrente. Puoi eliminare il file `4DDS.tar` per liberare spazio, ma ricorda che potrai recuperarlo nuovamente estraendolo da `4DDS.zip`.

### Come creare uno scenario e farlo partire

1. **Avviare l'ambiente 4DDS:**

   - Per accedere all'ambiente 4DDS, apri **PowerShell** e digita il comando:
     ```
     wsl -d 4DDS
     ```

2. **Accedere come utente ********`fourdds`********:**

   - Una volta avviato l'ambiente, devi accedere come utente `fourdds`. Digita i seguenti comandi:

     ```
     su fourdds
     cd
     ```

   - La password per l'utente `fourdds` è: `4DDS1234`

3. **Installare Visual Studio Code:**

   - Per lavorare più comodamente con il codice, è consigliabile installare **Visual Studio Code**. Segui le istruzioni presenti al [link ufficiale](https://code.visualstudio.com/docs/remote/wsl) per installare VS Code e abilitarlo a funzionare con WSL.
   - Utilizza la cartella `/home/fourdds` come workspace folder di VS Code per un accesso facilitato ai file di progetto.

4. **Installare Px4:**

   - Segui i seguenti passaggi per installare Px4 nell'ambiente 4DDS:

     ```
     cd
     git clone https://github.com/PX4/PX4-Autopilot.git --recursive
     bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
     cd PX4-Autopilot/
     make px4_sitl
     ```

   - **Nota Bene:** Se il comando `git` non funziona immediatamente, potrebbe essere necessario disinstallarlo e reinstallarlo per risolvere eventuali problemi di configurazione.

5. **Installare ROS2:**

   - Per l'integrazione con i droni e per il coordinamento avanzato, installa **ROS2** seguendo questi passaggi:
     ```
     sudo apt update && sudo apt install locales
     sudo locale-gen en_US en_US.UTF-8
     sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
     export LANG=en_US.UTF-8
     sudo apt install software-properties-common
     sudo add-apt-repository universe
     sudo apt update && sudo apt install curl -y
     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
     sudo apt update && sudo apt upgrade -y
     sudo apt install ros-humble-desktop
     sudo apt install ros-dev-tools
     source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
     ```
   - **Installazione delle dipendenze Python:**
     ```
     pip install --user -U empy==3.3.4 pyros-genmsg setuptools
     ```

6. **Installare Micro XRCE-DDS Agent & Client:**

   - Per la comunicazione tra Px4 e ROS2, è necessario installare il **Micro XRCE-DDS Agent & Client**:
     ```
     git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
     cd Micro-XRCE-DDS-Agent
     mkdir build
     cd build
     cmake ..
     make
     sudo make install
     sudo ldconfig /usr/local/lib/
     ```


7. **Installare Gazebo:**

   - **Gazebo** è necessario per la simulazione dei droni. Segui questi passaggi per installarlo:
     ```
     sudo apt remove gz-harmonic
     sudo apt install aptitude
     sudo aptitude install gazebo libgazebo11 libgazebo-dev
     ```
   - **Compilazione di Gazebo:**
     ```
     cd /path/to/PX4-Autopilot
     make px4_sitl gazebo-classic
     ```

### Conclusioni

Seguendo questi passaggi, avrai installato e configurato l'ambiente 4DDS su Windows utilizzando WSL. Questo ti permetterà di lavorare con Px4, ROS2 e Gazebo per sviluppare e testare scenari avanzati di simulazione e controllo di droni autonomi. Se incontri difficoltà durante il processo, assicurati di verificare eventuali errori riportati nella console e, se necessario, consulta la documentazione ufficiale di ciascun componente:

- [Documentazione per l'installazione di Px4, ROS2 e Micro XRCE-DDS](https://docs.px4.io/main/en/ros2/user_guide)
- [Documentazione per l'installazione di Gazebo](https://docs.px4.io/main/en/sim_gazebo_classic/)
