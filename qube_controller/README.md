# Qube Controller Package (Styringspakke for Qube)

`qube_controller` pakken er brukes for styring av Quanser Qube-motoren ved bruk av en PID-regulator. Den muliggjør presis posisjonskontroll og støtter både simulerings og virkelighet (hardware mode).

## Funksjoner
- **PID-regulator**: Implementerer en konfigurerbar PID-regulator for motorposisjon
- **Oppstartsintegrasjon**: Enkel konfigurasjon og oppstart via ROS 2-oppstartsfiler
- **Parametrisering**: Fullt parametrisert for fleksibilitet i innstilling og testing
- **Sanntids tilbakemelding**: Publiserer og logger leddtilstander og styrekommandoer

## Mappestruktur
```
qube_controller/
├── launch/controller.launch.py         # Oppstartsfiler for kontrolleren
├── qube_controller/pid_controller.py   # Kildekode for PID-regulatoren
├── resource/                           # 
└── test/                               # Test filer
package.xml                             # Definerer data og avhengigheter for ROS-pakken som brukes av ROS-byggesystemet
setup.py                                # Definere Python pakker og entry points til ROS 2 noder
```
## Avhengigheter

### ROS 2-biblioteker
Følgende ROS 2-biblioteker brukes i denne pakken:
- `rclpy`: ROS 2 Python-klientbibliotek for noder og ROS 2-interaksjon
- `rcl_interfaces.msg.SetParametersResult`: For parameterhåndtering i ROS 2
- `sensor_msgs.msg.JointState`: For abonnement på joint state updates
- `std_msgs.msg.Float64MultiArray`: For publisering publishing velocity control commands.

## Bruk

### Kjøring av kontrolleren
For å starte kontrolleren med standardparametere:
```bash
cd ~/ros2_qube
colcon build --packages-select qube_controller
source install/setup.bash  # Linux
ros2 launch qube_controller controller.launch.py
```

### Konfigurerbare parametere
Parametere kan overstyres i oppstartsfilen eller via kommandolinjen, inkludert:
- **`target_position`**: Ønsket motorposisjon 
- **`kp`, `ki`, `kd`**: Innstillingsparametere for PID-regulator
- **`max_velocity`**: Maksimal hastighetskommando for motoren
- **`deadband`**: Kompensasjon for motor-dødband

Eksempel på oppstart med egendefinerte parametere:
```bash
ros2 launch qube_controller controller.launch.py kp:=1.5 ki:=0.1 kd:=0.05 target_position:=3.14
```
Bruk av `ros2 param` kommand for å endere parameters.
Example:
```bash
ros2 param set /pid_controller kp 1.5
ros2 param set /pid_controller ki 0.5
ros2 param set /pid_controller kd 0.005
ros2 param set /pid_controller target_position 3.16
```


### Tilleggsopplysninger
- Pakken er optimalisert for ROS 2 jazzy/rolling
- Krever at `qube_description`, `qube_driver` og `qube_bringup`-pakkene er tilgjengelig i samme workspace
- Støtter både simulerte og fysiske Qube-enheter via parameteren `use_sim_time`
