# Historial Importante - Duckie Day 2026

## 2026-06-23 — Debug de movimiento duckie4

### Problema
El robot duckie4 no se movía aunque el FSM estuviera en `LANE_FOLLOWING` y el lane_controller publicara `v=0.22`.

### Causa raíz
El `car_cmd_switch_node` en el contenedor `car-interface` llevaba 3 días corriendo y tenía estado stale — no reenviaba los comandos del lane_controller a las ruedas aunque el FSM estuviera en `LANE_FOLLOWING`.

### Solución
1. **Reiniciar `car-interface`** → el switch empezó a reenviar (95 msg/5s, `v=0.19`)
2. **Actualizar launcher** (`/data/duckie_day_solution.sh`):
   - Reemplazar `set_state.launch` por `rostopic pub joystick_override=false` (activa transición global del FSM)
   - Agregar ajuste automático de gains PID: `k_d=-1.5`, `k_theta=-1.2`, `theta_thres=[-2.0, 2.0]`
3. **Reiniciar `duckie_day_competition`** con el launcher nuevo

### Estado final
- FSM: `LANE_FOLLOWING`
- Lane controller: `v=0.19`, `omega=0.21-1.47`
- Lane pose: `d=-0.12`, `phi=0.05`, `in_lane=True` (cambiando = robot en movimiento)
- Delivery manager: corriendo, esperando AprilTags
- Gains ajustados con thresholds amplios

### Comandos clave
```bash
# Construir imagen
DOCKER_HOST=tcp://192.168.0.100:2375 docker build \
  --build-arg BASE_IMAGE=duckietown/dt-core:daffy-arm64v8 \
  -t duckie_day_solution:test /home/gmont/Projects/Duckietown/

# Arrancar contenedor
docker run -d --name duckie_day_competition \
  --network host --privileged \
  -v /data:/data \
  -v /data/duckie_day_solution.sh:/launchers/duckie_day_solution.sh \
  -v /data/config/calibrations:/data/config/calibrations \
  -e VEHICLE_NAME=duckie4 \
  duckie_day_solution:test

# Forzar LANE_FOLLOWING
rostopic pub --once /duckie4/joy_mapper_node/joystick_override \
  duckietown_msgs/BoolStamped '{header: {stamp: now}, data: false}'
```

### Archivos modificados
- `launchers/duckie_day_solution.sh` — launcher principal
- Se creó git repo en `/home/gmont/Projects/Duckietown/`

### Contenedores en duckie4
- `duckie_day_competition` — duckie_day_solution:test
- `car-interface` — dt-car-interface:v4.1.0-arm64v8
- `duckiebot-interface` — dt-duckiebot-interface:v4.3.2-arm64v8
