
# 🦆 Duckie Day 2026 - Puerto Montt

(Creador: Patricio Carvajal)

*"Donde tu Duckiebot deja de ser un patito decorativo y se convierte en un ingeniero salmonero"*

**¿Qué es esto?**
Somos estudiantes de Duoc UC, sede Puente Alto de la region Metropolitana, y nos metimos al Duckie Day 2026.
La cosa va así: tenemos que programar un Duckiebot (un robot con forma de patito) para
que recorra una pista simulando un centro de cultivo de salmones. El robot tiene que
hacer 3 entregas de alimento en distintas estaciones, detectadas por códigos QR.

**Fecha del evento:** Jueves 25 de junio 2026, 09:00-14:00 hrs
**Lugar:** Duoc UC Sede Puerto Montt, Egaña 651
**Organiza:** Escuela de Informática y Telecomunicaciones
**Sponsors:** MOWI (los reyes del salmón) y Duck House (tienda de patitos de goma)

---

## 📋 Cómo evaluán (la rúbrica)

| Qué miran | Cómo lo miden |
|-----------|---------------|
| ⏱ Tiempo | Lo que te demores en completar la ruta |
| 🎯 Precisión | Que hagas las entregas en el orden correcto según el JSON |
| 🛡 Seguridad | Que no chocai, no te salgas de la pista, no hagai movimientos raros |
| 🔧 Robustez | Que el código no se caiga a la primera de cambios |

**Penalizaciones (esto duele):**
- Cada choque: **+10 segundos** a tu tiempo
- Salirse de la pista: **+15 segundos**
- No completar todas las entregas: te bajan la nota

---

## 🖥️ 1. EMULACIÓN EN EL PC (para programar y probar)

Esto corre en tu compu usando Docker. No necesitas el robot físico.

### Requisitos
- Linux con Docker instalado
- El Duckietown Shell (`dts`) configurado
- ~10GB de espacio libre (las imágenes Docker pesan)

### 1.1 Ejecutar el Simulador (Menú Interactivo)

Hemos unificado todo en un solo lanzador con interfaz. Este menú te permite probar el robot manualmente o lanzar la inteligencia artificial que competirá.

```bash
cd ~/Projects/Duckietown
./launchers/duckie_day_menu.sh
```

**El menú ahora tiene 5 opciones:**
- **[1] Clasificatoria - Alerta Sanitaria:** Simula detección de piojo de mar. El robot detecta QR-05, enciende luces traseras 3 veces, retorna a base, espera 10s, reanuda.
- **[2] Semifinal - Trazabilidad Completa:** Recorre secuencialmente Piscicultura Puelo (QR-01) → Jaulas Isla Huar (QR-02) → Planta Calbuco (QR-03). Orden estricto con luces frontales y paradas de 8s.
- **[3] Final - Inspección Visión Artificial:** Clasifica objetos como aprobados o defectuosos en zona de inspección (QR-06), desvía a zona correspondiente (QR-07/QR-08), retorna a base.
- **[4] Control Manual:** Manejas con las flechitas del teclado, `SHIFT` para el turbo y `ESPACIO` para frenar.
- **[5] Competencia Automática (demo):** El robot se maneja solo siguiendo la línea blanca y entregando el alimento según el archivo JSON (modo anterior).

**📸 Controles de Cámara (funcionan en ambos modos):**
- Apreta **C** en cualquier momento para cambiar entre 1ra persona (lo que ve el robot) y 3ra persona (cámara dinámica desde atrás viendo el modelo 3D del patito).
- Apreta **R** para resetear la simulación si te chocaste muy feo.
- Apreta **ESC** para cerrar la ventana y volver al menú principal.

### 1.2 ¿Cómo nos aseguramos ganar según la Rúbrica? (La IA)

Nuestro modo de Competencia Automática no solo avanza, está hecho a medida para el Duckie Day:
1. **Precisión:** Lee `config/deliveries.json` y ordena las misiones matemáticamente por su `priority`. ¡Cumplimos la logística exacta!
2. **Seguridad Operativa:** El controlador PID tiene frenado dinámico. Si el volante gira muy brusco (viene una curva cerrada), la matemática le quita aceleración para no salirse de la pista y evitar los 15s de penalización.
3. **Robustez (Manejo de Excepciones):** Si llega a una intersección donde pierde la línea de visión, un bloque `try/except` evita que el sistema crashee; simplemente baja la velocidad hasta recuperar la línea.
4. **Circuito Largo:** Las entregas están espaciadas matemáticamente para que el robot se vea obligado a demostrar que sabe doblar y navegar por casi todo el mapa antes de terminar.
5. Al terminar, la terminal imprime una **Evaluación Oficial** imitando los criterios del jurado (Tiempo, Colisiones, Excepciones manejadas y Precisión).

---

## 🦆 2. DUCKIEBOT FÍSICO (el día de la competencia)

La simulación en el PC es una cosa, pero el robot real en la pista es otra. El robot usa **ROS (Robot Operating System)**, que es el estándar en robótica. Esto significa que no podemos simplemente ejecutar nuestro script de Python. ¡Pero no te asustes! Lo he preparado todo para que sea (casi) un copiar y pegar.

Nuestra lógica de competencia (seguimiento de carril + detección de señales de PARE) ha sido portada al archivo:
`packages/duckie_day_solution/src/duckie_day_node.py`.

Este "nodo" de ROS es el nuevo cerebro del robot.

### ⚠️ ¡ADVERTENCIA CRÍTICA! CALIBRACIÓN EN EL MUNDO REAL

Esto es lo más importante: **los valores que funcionan en el simulador NO funcionarán igual en el robot real.**
-   **Iluminación:** La luz del entorno real cambia cómo la cámara ve los colores. El rojo de la señal de PARE puede verse diferente.
-   **Física:** El motor del robot, el roce de las ruedas y la inercia son distintos.

**¿Qué significa esto?** Tendrás que **re-calibrar** dos cosas directamente en el código de `duckie_day_node.py`:
1.  **Valores del PID:** Las constantes `kp`, `ki`, `kd` en `self.pid_angle` y `self.pid_distance` probablemente necesiten ajuste. Si el robot zigzaguea mucho, baja `kp`. Si no dobla lo suficiente, súbelo.
2.  **Detección de Color:** Los rangos `lower_red1`, `upper_red1`, etc., en la función `detect_stop_sign` son para el simulador. Para ajustarlos al mundo real, puedes usar un "color picker" online o una herramienta de OpenCV para encontrar el rango HSV del color rojo bajo la luz de la sala de competencia.

### 2.1 Estructura de archivos para ROS

El proyecto ya quedó preparado como proyecto Duckietown (`.dtproject`) y como workspace ROS. La estructura relevante es:

```
Projects/Duckietown/
├── .dtproject
├── Dockerfile
├── config/
│   └── deliveries.json
├── launchers/
│   ├── duckie_day_menu.sh
│   └── duckie_day_solution.sh
└── packages/
    └── duckie_day_solution/
        ├── Dockerfile
        ├── launch/
        │   ├── default.launch
        │   └── duckie_day_competition.launch
        ├── src/
        │   ├── duckie_day_node.py
        │   ├── delivery_manager.py
        │   └── safety_monitor.py
        ├── CMakeLists.txt
        └── package.xml
```

### 2.2 Validar antes de competir

Desde el PC, en la raíz del proyecto:

```bash
cd ~/Projects/Duckietown
PYTHONPYCACHEPREFIX=/tmp/duckie_pycache python3 -m py_compile \
  scripts/duckie_day_simulation.py \
  scripts/duckie_day_menu.py \
  packages/duckie_day_solution/src/duckie_day_node.py \
  packages/duckie_day_solution/src/delivery_manager.py \
  packages/duckie_day_solution/src/safety_monitor.py
```

Validación Docker/ROS local:

```bash
cd ~/Projects/Duckietown
docker build -t duckie_day_project:test .
docker run --rm \
  -e VEHICLE_NAME=duckiebot \
  -e ROBOT_TYPE=duckiebot \
  -e ROBOT_CONFIGURATION=DB21M \
  --entrypoint /bin/bash \
  duckie_day_project:test \
  -lc 'source /opt/ros/noetic/setup.bash && source /code/catkin_ws/devel/setup.bash && roslaunch duckie_day_solution default.launch --nodes'
```

El build probado localmente compila 32 paquetes ROS y deja `duckie_day_solution` sin errores.

### 2.3 Subir y Ejecutar Nuestro Código (Paso a Paso)

**Paso 0: Conéctate a la red WiFi del Duckiebot** y confirma el nombre exacto del robot.

**Paso 1: Copiar el JSON oficial.**

Cuando entreguen el JSON de competencia, reemplaza:

```bash
~/Projects/Duckietown/config/deliveries.json
```

El código ordena las entregas por `priority`, que es lo que pide la rúbrica.

**Paso 2: Construir imagen con Duckietown Shell.**

En este PC, el perfil `dts daffy` reconoce `dts code build`, pero exige credenciales Docker para `docker.io`. Si ya tienes credenciales:

```bash
cd ~/Projects/Duckietown
dts challenges config --docker-server docker.io --docker-username TU_USUARIO --docker-password TU_PASSWORD
dts code build -C ~/Projects/Duckietown --no-pull -L duckie_day_solution
```

Si no tienes credenciales, el build manual local funciona:

```bash
cd ~/Projects/Duckietown
docker build -t duckie_day_project:test .
```

**Paso 3: Ejecutar en el Duckiebot físico.**

El `dts code run` instalado en este PC es un stub y no despliega al robot. Para ejecutar el contenedor ya construido en el Duckiebot usa SSH al robot o el flujo `dts code run` de una instalación Duckietown Shell completa.

Por SSH, una vez que la imagen esté en el Duckiebot:

```bash
ssh duckie@NOMBRE_DUCKIEBOT.local
docker run --rm -it --net=host --privileged \
  -e VEHICLE_NAME=NOMBRE_DUCKIEBOT \
  -e ROBOT_TYPE=duckiebot \
  -e ROBOT_CONFIGURATION=DB21M \
  -e DELIVERIES_PATH=/data/deliveries.json \
  duckie_day_project:test
```

Si el robot usa otra configuración (`DB19`, `DB21J`, etc.), cambia `ROBOT_CONFIGURATION`.

### 2.4 Comandos de Emergencia

- **Detener el contenedor:** `docker ps` y luego `docker stop <container_id>`.
- **Ver logs:** `docker logs <container_id>`.
- **Bajar velocidad si se sale de pista:** edita `base_speed` en `packages/duckie_day_solution/launch/default.launch` o parámetro `~base_speed` del nodo.
- **Ajustar detección de PARE/entrega:** calibra los rangos HSV en `detect_stop_sign()` dentro de `packages/duckie_day_solution/src/duckie_day_node.py`.

---

## 📁 Estructura del proyecto

```
Projects/Duckietown/
│
├── config/
│   └── deliveries.json        # ⭐ ESTE ARCHIVO LO ENTREGA LA COMPETENCIA
│
├── Dockerfile                 # Build ROS completo para Duckiebot
├── .dtproject                 # Metadata para dts code build
│
├── scripts/
│   └── duckie_day_menu.py     # Menú para SIMULACIÓN (PC)
│
├── packages/
│   └── duckie_day_solution/   # Paquete ROS para el ROBOT FÍSICO
│       ├── Dockerfile           # 🆕 Define el entorno del robot
│       ├── launch/
│       │   └── default.launch   # 🆕 Orquesta los nodos de ROS
│       ├── src/
│       │   └── duckie_day_node.py # ✅ NUESTRO NODO CON TODA LA LÓGICA
│       └── package.xml
│
├── launchers/
│   └── duckie_day_menu.sh     # Lanzador rápido de la simulación (PC)
│
└── README.md                  # Este archivo
```

---

## 📄 El archivo deliveries.json (ESTE ES LA CLAVE)

Este archivo define las entregas. **La competencia te va a pasar uno el día del evento**
con las estaciones reales. Tú solo tienes que copiarlo a `config/deliveries.json`.

```json
{
  "competition": "Duckie Day 2026 - Puerto Montt",
  "deliveries": [
    {
      "id": "QR001",
      "station": "P1",
      "feed": "Alimento Salmón Iniciador A",
      "kg": 50.0,
      "priority": 1
    },
    {
      "id": "QR002",
      "station": "P2",
      "feed": "Alimento Salmón Crecimiento B",
      "kg": 30.0,
      "priority": 2
    },
    {
      "id": "QR003",
      "station": "P3",
      "feed": "Alimento Salmón Engorda C",
      "kg": 40.0,
      "priority": 3
    }
  ],
  "rules": {
    "order": "sequential",
    "max_time_seconds": 300,
    "collision_penalty_seconds": 10,
    "track_exit_penalty_seconds": 15
  }
}
```

---

## ⚙️ Cómo funciona por dentro (pa' los curiosos)

El cerebro del Duckiebot tiene 3 módulos:

```
                    ┌──────────────────────┐
                    │     ROS Master       │
                    │  (orquestador)       │
                    └──────────┬───────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
   ┌────▼────────┐      ┌─────▼──────────┐     ┌─────▼──────────┐
   │ Lane        │      │ AprilTag       │     │ Delivery       │
   │ Following   │      │ Detector       │     │ Manager        │
   │ (sigue la   │      │ (detecta QR)   │     │ (gestiona      │
   │  línea)     │      │                │     │  entregas)     │
   └────┬────────┘      └─────┬──────────┘     └─────┬──────────┘
        │                     │                      │
        └─────────────────────┼──────────────────────┘
                              │
                     ┌────────▼────────┐
                     │  Safety Monitor │
                     │  (evita que     │
                     │  choques)       │
                     └────────┬────────┘
                              │
                     ┌────────▼────────┐
                     │  Car Cmd        │
                     │  (v = vel,      │
                     │   ω = giro)     │
                     └─────────────────┘
```

**El PID Controller** es un loop de control que ajusta la dirección del robot:
- Mide qué tan lejos está del centro del carril (`dist`)
- Mide qué tan desviado va (`angle`)
- Calcula cuánto girar para corregir
- Ajusta la velocidad según la curva

---

## 🏆 Tips pal día de la competencia (de estudiante a estudiante)

1. **LLEGA TEMPRANO.** Dicen que a las 10:00 abren la pista pa' practicar. No lleguís volando a las 11.
2. **Carga el Duckiebot.** Literal, que tenga batería. Suena obvio pero siempre alguien se olvida.
3. **El JSON te lo dan allá.** No inventís tus propias estaciones. Copia el archivo que te entreguen y pegalo en `config/deliveries.json`.
4. **Prueba la conexión WiFi primero.** No querís estar peleando con la red 5 minutos antes de competir.
5. **Ajusta la velocidad en la práctica.** Si ves que el robot se sale en las curvas, baja la velocidad. Mejor lento y preciso que rápido y en la cuneta.
6. **Parada de emergencia.** Ten listo `docker ps` y `docker stop <container_id>`.
7. **Respira.** Es solo un patito robot repartiendo comida de salmón. Pero gana el que lo hace mejor.

---

## 🔧 Solución de problemas comunes

| Problema | Solución |
|----------|----------|
| `xhost: not found` | `sudo pacman -S xorg-xhost` (Arch) / `sudo apt install x11-xserver-utils` (Debian) / `sudo dnf install xorg-x11-server-utils` (Fedora) |
| No se ve la ventana | Revisa que `$DISPLAY` no esté vacío: `echo $DISPLAY` |
| Container se cae altiro | `docker ps -a` y luego `docker logs <container_id>` pa' ver el error |
| Error de fontconfig | Ya lo instala el script automáticamente |
| No reconoce el JSON | Revisa que el archivo esté en `config/deliveries.json` con el formato correcto |
| `dts code build` pide Docker login | Configura credenciales con `dts challenges config ...` o usa `docker build -t duckie_day_project:test .` |

---

*Hecho con 💻 y ☕ por estudiantes de Duoc UC, Escuela de Informática y Telecomunicaciones, Sede Puerto Montt.*
*Si llegaste hasta acá, ya tenís más ventaja que la mitad de los equipos. ¡Buena suerte! 🦆*
