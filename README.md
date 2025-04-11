### Quanser Qube ROS 2-pakke

Denne pakken gir et ROS2 for å styre  Quanser Qube. Den inkluderer nødvendige noder for å kontrollere Qube ved hjelp av en PID-regulator og visualisere robotens tilstand i RViz.
Prosjektet inneholder qube_bringup, qube_driver, qube_discription og qube_controller pakker.

## Innholdsfortegnelse

- [Installasjon](#installasjon)
- [Bruk](#bruk)
  - [Koble til Qube](#koble-til-qube)
  - [Kjør RViz](#kjør-rviz)
  - [Kjøre nodene](#kjøre-nodene)
  - [Angi parametere](#angi-parametere)

## Installasjon

### Avhengigheter

Sørg for at følgende avhengigheter er installert:

- ROS 2 (Galactic eller nyere)
- rviz2 for visualisering
- ros2 controll for kontrollgrensesnitt
- Python 3.8 eller nyere
### Installere pakken

Klone depotet og bygg pakken:

```bash
git clone https://github.com/TechNinja88/qube
cd ros2_qube
colcon build --symlink-install
source install/setup.bash
```

## Bruk

### Koble til Qube

1. Koble Quanser Qube til datamaskinen via USB.
2. Finn riktig USB-port. Typisk vil det være noe som `/dev/ttyACM1`.
3. Sørg for at korrekt firmware er lastet opp til Qube ved hjelp av Arduino IDE.

### Kjør RViz
For å visualisere Qube i RViz, kjør følgende kommando:

```bash
ros2 launch qube_bringup bringup.launch.py simulation:=true
```

### Kjøre nodene

For å starte PID-regulatoren og koble til Qube, bruk følgende kommando:



```bash
ros2 launch qube_bringup bringup.launch.py simulation:=false
ros2 launch qube_controller controller.launch.py
```

Dette vil starte nødvendige noder for å kontrollere Qube og visualisere tilstanden dens.

### Angi parametere

Du kan angi PID-parameterne med `ros2 param set`-kommandoen:

```bash
ros2 param set /pid_controller target_position 1.57
ros2 param set /pid_controller kp 2.0
ros2 param set /pid_controller ki 0.2
ros2 param set /pid_controller kd 0.1
```

**Parametere:**
- `kp`: Proporsjonalforsterkning
- `ki`: Integralforsterkning
- `kd`: Derivativforsterkning
- `target_position`: Ønsket posisjon for rotasjonsleddet
- `max_velocity`: Maksimal hastighetskommando (rad/s)
- `max_effort`: Maksimal kraftkommando (Nm)
## gruppe deltakere
Christian gimse 
Wahid Sediqi
