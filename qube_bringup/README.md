# qube_bringup  

En ROS 2-pakke for å starte opp Quanser Qube-maskinvaren eller simuleringsmiljøet.  

## Oversikt  

`qube_bringup` pakken inkluder oppstartsfiler og konfigurasjon for å starte et Quanser Qube, Den støtter både simulerings og ekte maskinvare med konfigurerbare parametere for maskinvarekommunikasjon.  

## Funksjoner  

- Komplett systemoppstart med én enkelt oppstartsfil  
- Støtte for både simulering og ekte maskinvare  
- Konfigurerbare maskinvareparametere (enhet, baudrate, port)  
- RViz-visualisering  
- Integrasjon med ROS 2 Control-rammeverket  

## Avhengigheter

`qube_bringup`-pakken er avhengig av følgende ROS 2-pakker:

- `qube_description`: Tilbyr Xacro beskrivelsen for Quanser Qube.
- `qube_driver`: Qube-maskinvaren eller simulasjonen.
- `robot_state_publisher`: Publiserer robotens tilstand basert på URDF-en.
- `controller_manager`: Håndterer ROS 2-kontrollerne.
- `rviz2`: Visualiserer Qube-en i RViz.

Disse pakkene er inkludert i ros2_qube.


## Installasjon
### Bygging fra kildekode  

```bash  
# Opprett et arbeidsområde (hvis du ikke allerede har ett)  
mkdir -p ~/ros2_qube/src  
cd ~/ros2_qube/src  

# Klon repositoriet  
git clone https://github.com/TechNinja88/ros2_qube.git .  

# Installer avhengigheter  
cd ~/ros2_qube  
rosdep install --from-paths src --ignore-src -r -y  

# Bygg pakkene  
colcon build --symlink-install  
source install/setup.bash  
```  

### Bruk  
    
For å starte systemet i simuleringsmodus:  
```bash  
ros2 launch qube_bringup bringup.launch.py simulation:=true  
```  

For å starte med ekte maskinvare (endre enhet og baudrate etter behov):  
```bash  
ros2 launch qube_bringup bringup.launch.py simulation:=false 
```  

## Konfigurasjon  

### Oppstartsargumenter  

| Argument      | Standardverdi   | Beskrivelse                          |  
|---------------|-----------------|---------------------------------------|  
| `simulation`  | `true`          | Aktiver simuleringsmodus              |  
| `device`      | `/dev/ttyUSBAX` | Seriell enhet for maskinvaretilkobling|  
| `baud_rate`   | `115200`        | Kommunikasjonshastighet for seriell   |  
| `use_rviz`    | `true`          | Aktiver RViz-visualisering            |