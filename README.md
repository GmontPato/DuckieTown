# DuckieTown

Configuracion y archivos del duckie4 fisico para auto-conduccion (lane following).

## Requisitos

- Duckiebot con Jetson Nano (arm64v8) o laptop/PC con Docker
- Duckietown Shell (`dts`) instalado
- Docker 20.10+

## Imagenes Docker

Las imagenes se descargan de Docker Hub:

```bash
bash images/pull.sh
```

## Calibraciones

Las calibraciones de la camara del duckie4 estan en `config/calibrations/`.
Copiar al duckie:

```bash
DOCKER_HOST=tcp://duckie4.local:2375 docker cp config/calibrations/camera_intrinsic/duckie4.yaml ros:/data/config/calibrations/camera_intrinsic/
DOCKER_HOST=tcp://duckie4.local:2375 docker cp config/calibrations/camera_extrinsic/duckie4.yaml ros:/data/config/calibrations/camera_extrinsic/
DOCKER_HOST=tcp://duckie4.local:2375 docker cp config/calibrations/kinematics/duckie4.yaml ros:/data/config/calibrations/kinematics/
```

## Archivos modificados

Los archivos con boundary enforcement y obstacle avoidance estan en `config/lane_controller/`.
Copiar al duckie:

```bash
DOCKER_HOST=tcp://duckie4.local:2375 docker cp config/lane_controller/controller.py ros:/data/
DOCKER_HOST=tcp://duckie4.local:2375 docker cp config/lane_controller/lane_controller_node.py ros:/data/
```

## Iniciar auto-conduccion

```bash
bash deploy/run_lane_following.sh duckie4
```

Esto:
1. Inicia el contenedor `lane_following` con los archivos modificados montados
2. Activa el FSM a `LANE_FOLLOWING`
3. Aplica los parametros PID tuneados (v_bar=0.30, k_d=-10, k_theta=-8)

### Para detener

```bash
DOCKER_HOST=tcp://duckie4.local:2375 docker stop lane_following
```

### Para solo aplicar parametros (si el contenedor ya esta corriendo)

```bash
bash deploy/set_params.sh duckie4
```

## Modificaciones respecto al Duckietown original

### 1. Boundary Enforcement (`controller.py`)
Fuerza repulsiva que evita que el duckie traspase las lineas:

- Linea amarilla (izquierda): si `d < -0.10` -> fuerza positiva (gira derecha)
- Linea blanca (derecha): si `d > 0.10` -> fuerza negativa (gira izquierda)
- Margen suave de 0.04m antes del limite para correccion gradual

### 2. Obstacle Avoidance (`lane_controller_node.py`)
Cuando se detecta un obstaculo:

- Desplaza `d_offset` +0.06m a la derecha (dentro del carril)
- Reduce velocidad al 25%
- Cuando el obstaculo se despeja, restaura el offset

### 3. PID Tuneado (params.yaml)
- Velocidad aumentada de 0.19 a 0.30 m/s (+60%)
- Ganancias proporcionales mas agresivas para estabilidad a alta velocidad

## Parametros completos

Ver `params.yaml` para la configuracion completa de todos los parametros.
