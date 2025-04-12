# Qube Description Package (Beskrivelsespakke for Qube)

`qube_description`-pakken representerer URDF (Unified Robot Description Format) og andre ressurser for å representere Quanser Qube i simulerings- og visualiseringsverktøy som RViz.

## Funksjoner
- **URDF-modell**: Definerer fysiske og visuelle egenskapene til Quanser Qube.
- **Xacro-integrasjon**: Modulære og parametriserte definisjoner ved bruk av Xacro-filer.
- **Simuleringsstøtte**: Muliggjør bruk i Gazebo og andre simuleringsmiljøer.
- **Visualisering i RViz**: Enkel visualisering av modellen i RViz for feilsøking og demonstrasjon.

## Mappestruktur
```
qube_description/
├── config/qube_view.rviz         # Oppstartsfiler for RViz 
├── launch/view_qube.launch.py    # launch fil til å kjøre modellen
├── qube_description/             # 3D-modeller og teksturer for visualisering
├── resource/                     # tomt mappe
├── test/                         # ligger noe test filer
urdf/
├── qube.macro.xacro       # Definerer selve Qube-modellen (dimensjoner, deler, farger etc.)
└── qube.urdf.xacro        # Bruker (inkluderer) qube.macro.xacro - fungerer som en "hovedfil"
package.xml             # Definerer data og avhengigheter for ROS-pakken som brukes av ROS-byggesystemet (colcon)
README.md               # Kort representasjon om pakken
setup.py                # Er byggeskript for pakken og sørger for at alle ressursfiler URDF og launch blir korrekt installert
```
## Bruk
### Visualisering i RViz
For å se Qube-modellen i RViz, kjør:
```bash
cd ~/ros2_qube
colcon build --packages-select qube_description
source install/setup.bash
ros2 launch qube_description view_qube.launch.py
```

## Avhengigheter
- ROS 2 jazzy eller nyere
- `xacro`-pakke for parametrisert URDF
- `rviz2` for visualisering
