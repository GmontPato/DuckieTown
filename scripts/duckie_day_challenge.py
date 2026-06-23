#!/usr/bin/env python3
"""
Duckie Day 2026 - Desafios Oficiales
Clasificatoria, Semifinal y Final
"""
import json
import os
import sys
import time
import math
import numpy as np
import pyglet
from pyglet.window import key
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv

try:
    import cv2
except ImportError:
    cv2 = None

QR_DIR = os.environ.get("QR_DIR", "/data/qr_codes")
CHALLENGES_PATH = os.environ.get("CHALLENGES_PATH", "/data/challenges.json")
QR_POSITIONS_PATH = os.environ.get("QR_POSITIONS_PATH", "/data/qr_positions.json")
MAP_NAME = os.environ.get("MAP_NAME", "duckie_day_2026")

CAM_NAMES = {0: "1ra Persona", 1: "3ra Persona"}

TILE_SIZE = 0.585
QR_DETECTION_RADIUS = 0.25

DEFAULT_QR_POSITIONS = {
    "QR-00": [1.5, 5.5],
    "QR-01": [5.5, 5.5],
    "QR-02": [6.5, 1.5],
    "QR-03": [2.5, 1.5],
    "QR-05": [6.5, 3.5],
    "QR-06": [3.5, 4.5],
    "QR-07": [2.5, 5.5],
    "QR-08": [4.5, 4.5],
}


def load_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default if default is not None else {}


OFFICIAL_CONFIGS = {
    "QR-05": {"qr_id": "QR-05", "nombre_punto": "Alerta Sanitaria", "tipo_luz": "trasera", "repeticiones": 3, "tiempo_detencion": 10},
    "QR-01": {"qr_id": "QR-01", "nombre_punto": "Piscicultura Puelo", "tipo_luz": "frontal", "repeticiones": 1, "tiempo_detencion": 8},
    "QR-02": {"qr_id": "QR-02", "nombre_punto": "Jaulas Solar Isla Huar", "tipo_luz": "frontal", "repeticiones": 1, "tiempo_detencion": 8},
    "QR-03": {"qr_id": "QR-03", "nombre_punto": "Planta de Proceso Calbuco", "tipo_luz": "frontal", "repeticiones": 1, "tiempo_detencion": 8},
    "QR-06": {"qr_id": "QR-06", "nombre_punto": "Zona de Inspeccion", "tiempo_detencion": 8},
    "QR-07": {"qr_id": "QR-07", "nombre_punto": "Zona Aprobado", "tiempo_detencion": 5, "resultado": "aprobado"},
    "QR-08": {"qr_id": "QR-08", "nombre_punto": "Zona Defectuoso", "tiempo_detencion": 5, "resultado": "defectuoso"},
    "QR-00": {"qr_id": "QR-00", "nombre_punto": "Punto de Partida / Base", "tiempo_detencion": 5}
}

def load_qr_config(qr_id):
    path = os.path.join(QR_DIR, f"{qr_id}.json")
    cfg = load_json(path, None)
    if not cfg:
        cfg = OFFICIAL_CONFIGS.get(qr_id, {"qr_id": qr_id, "accion": "desconocido", "tiempo_detencion": 3})
    return cfg


def load_challenges():
    fallback = {
        "challenges": [
            {"id": 1, "nombre": "Alerta Sanitaria", "tipo": "clasificatoria", "qrs": ["QR-05"]},
            {"id": 2, "nombre": "Trazabilidad Completa", "tipo": "semifinal", "qrs": ["QR-01", "QR-02", "QR-03"], "restricciones": {"orden_estricto": True}},
            {"id": 3, "nombre": "Inspeccion con Vision Artificial", "tipo": "final", "qrs": ["QR-06", "QR-07", "QR-08", "QR-00"]}
        ]
    }
    return load_json(CHALLENGES_PATH, fallback)


def load_qr_positions():
    return load_json(QR_POSITIONS_PATH, DEFAULT_QR_POSITIONS)


def classify_object(image):
    """
    Clasifica objeto por visión artificial (HSV).
    Verde predominante -> aprobado; rojo predominante -> defectuoso.
    """
    if cv2 is None or image is None:
        return "aprobado"

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    green_pixels = cv2.countNonZero(green_mask)
    red_pixels = cv2.countNonZero(red_mask)

    if red_pixels > green_pixels and red_pixels > 500:
        return "defectuoso"
    return "aprobado"

def detect_stop_sign(image):
    if cv2 is None or image is None: return False
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < 500: continue
        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        if len(approx) == 8:
            return True
    return False



class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = self.last_error = 0.0

    def reset(self):
        self.integral = self.last_error = 0.0

    def compute(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.last_error) / max(dt, 0.01)
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return np.clip(out, -1.0, 1.0)


def print_detailed_result(challenge_num, nombre, solver, qrs):
    elapsed = time.time() - solver.challenge_start_time
    print("\n  ========================================")
    print(f"  🏆 EVALUACION DESAFIO {challenge_num}")
    print("  ========================================")
    print(f"  Desafio        : {nombre}")
    print(f"  Tiempo total   : {elapsed:.1f}s")
    print(f"  QRs visitados  : {len(solver.visited_qrs)}/{len(qrs)}")
    print(f"  Colisiones     : {solver.collisions}")
    if solver.clasificacion:
        print(f"  Clasificacion  : {solver.clasificacion.upper()}")
    print("  ========================================\n")


class ChallengeSolver:
    def __init__(self, challenge_cfg):
        self.cfg = challenge_cfg
        self.challenge_id = challenge_cfg.get("id", 0)
        self.state = "navegar"
        self.strict_order = challenge_cfg.get("restricciones", {}).get("orden_estricto", False)
        self.visited_qrs = set()
        self.wait_timer = 0.0
        self.done = False

        self.pid_angle = PIDController(kp=6.0, ki=0.0, kd=0.8)
        self.pid_dist = PIDController(kp=5.0, ki=0.01, kd=1.0)
        self.kp_angle_straight = 2.5
        self.kp_dist_straight = 2.0
        self.driving_mode = "STRAIGHT"
        self.base_speed = 0.4

        self.qr_positions = load_qr_positions()
        self.qr_configs = {}
        self.last_detected_qr = None
        self.total_reward = 0.0
        self.collisions = 0
        self.challenge_start_time = time.time()
        self.start_pos = None
        self.clasificacion = None
        self.blocked_warnings = set()
        self.route_resumed_after_alert = not self._requires_alert_resume()
        self.left_start_after_resume = False
        self.will_complete = False

        self.stop_sign_timer = 0.0
        self.current_v = 0.0
        self.current_omega = 0.0

        self._init_qr_queue(challenge_cfg)

    def _requires_alert_resume(self):
        return self.challenge_id == 1

    def _init_qr_queue(self, challenge_cfg):
        all_qrs = list(challenge_cfg.get("qrs", []))
        if self.challenge_id == 3:
            self.qr_queue = ["QR-06"]
            self.qr_index = 0
            self.final_qrs = all_qrs
        else:
            self.qr_queue = all_qrs
            self.qr_index = 0
            self.final_qrs = all_qrs

        for qr_id in set(all_qrs + ["QR-00"]):
            self.qr_configs[qr_id] = load_qr_config(qr_id)

    def reset(self):
        self.__init__(self.cfg)

    def _qr_world_pos(self, qr_id):
        qr_pos = self.qr_positions.get(qr_id)
        if qr_pos is None:
            return None
        return qr_pos[0] * TILE_SIZE, qr_pos[1] * TILE_SIZE

    def _distance_to_qr(self, pos, qr_id):
        world = self._qr_world_pos(qr_id)
        if world is None:
            return float("inf")
        # Corrección crucial: en Duckietown 3D, pos es [x, y, z] (y es altura)
        # El mapa 2D usa X y Z (profundidad). 
        dx = pos[0] - world[0]
        dz = pos[2] - world[1]
        return math.sqrt(dx * dx + dz * dz)

    def expected_qr(self):
        if self.qr_index < len(self.qr_queue):
            return self.qr_queue[self.qr_index]
        return None

    def check_near_qr(self, pos):
        candidates = []
        for qr_id, qr_pos in self.qr_positions.items():
            if qr_id in self.visited_qrs:
                continue
            dist = self._distance_to_qr(pos, qr_id)
            if dist < QR_DETECTION_RADIUS:
                candidates.append((dist, qr_id))

        if not candidates:
            return None

        candidates.sort(key=lambda x: x[0])
        nearest_qr = candidates[0][1]
        expected = self.expected_qr()

        if self.strict_order and expected and nearest_qr != expected:
            if nearest_qr not in self.blocked_warnings:
                print(f"  ⛔ BLOQUEADO: {nearest_qr} - debe visitar {expected} primero")
                self.blocked_warnings.add(nearest_qr)
            return None

        if expected and nearest_qr != expected:
            return None

        return nearest_qr

    def simulate_lights(self, cfg):
        tipo = cfg.get("tipo_luz", "frontal")
        repeticiones = cfg.get("repeticiones", 1)
        if tipo == "frontal":
            for i in range(repeticiones):
                print(f"  💡 Luces frontales ON  ({i + 1}/{repeticiones})")
        elif tipo == "trasera":
            for i in range(repeticiones):
                print(f"  🔴 Luces traseras ON  ({i + 1}/{repeticiones})")

    def is_near_start(self, pos):
        if self.start_pos is None:
            return False
        # Corrección crucial 3D a 2D
        dx = pos[0] - self.start_pos[0]
        dz = pos[2] - self.start_pos[2]
        return math.sqrt(dx * dx + dz * dz) < QR_DETECTION_RADIUS

    def drive(self, env, dt):
        try:
            lane = env.get_lane_pos2(env.cur_pos, env.cur_angle)
            dist, angle = lane.dist, lane.angle_rad
        except Exception:
            return np.array([0.12, 0.0])

        if abs(angle) > 0.22 and self.driving_mode == "STRAIGHT":
            self.driving_mode = "CURVE"
            self.pid_dist.reset()
        elif abs(angle) < 0.1 and self.driving_mode == "CURVE":
            self.driving_mode = "STRAIGHT"

        if self.driving_mode == "STRAIGHT":
            steering = (self.kp_angle_straight * angle) + (self.kp_dist_straight * dist)
        else:
            sa = self.pid_angle.compute(angle, dt)
            sd = self.pid_dist.compute(dist, dt)
            steering = sa + sd

        steering = np.clip(steering, -1.0, 1.0)
        speed = self.base_speed * max(0.35, 1.0 - abs(steering) * 0.7)
        return np.array([np.clip(speed, 0.0, self.base_speed), steering])

    def _can_complete_mission(self, pos):
        if self.challenge_id == 1:
            required = {"QR-05"}
            if not required.issubset(self.visited_qrs):
                return False
            if not self.route_resumed_after_alert or not self.left_start_after_resume:
                return False
            return self.is_near_start(pos)

        if self.challenge_id == 2:
            required = {"QR-01", "QR-02", "QR-03"}
            if not required.issubset(self.visited_qrs):
                return False
            return self.is_near_start(pos)

        if self.challenge_id == 3:
            required = {"QR-06", "QR-00"}
            if not required.issubset(self.visited_qrs):
                return False
            zone = "QR-07" if self.clasificacion == "aprobado" else "QR-08"
            if zone not in self.visited_qrs:
                return False
            return True

        return self.is_near_start(pos)

    def get_action(self, obs, env, dt=0.1):
        target_v = 0.0
        target_omega = 0.0

        if self.stop_sign_timer > 0:
            self.stop_sign_timer -= dt
            target_v = 0.0
            target_omega = 0.0
        elif self.wait_timer > 0:
            self.wait_timer -= dt
            target_v = 0.0
            target_omega = 0.0
            if self.wait_timer <= 0 and getattr(self, 'will_complete', False):
                self.state = "completar"
        else:
            pos = env.unwrapped.cur_pos

            if self.start_pos is None and self.state == "navegar":
                self.start_pos = list(pos)
                print(f"  📌 Punto de partida registrado: [{pos[0]:.2f}, {pos[1]:.2f}]")

            if detect_stop_sign(obs) and self.state == "navegar":
                print("  🛑 ¡Señal de PARE detectada! Frenando...")
                self.stop_sign_timer = 3.0
                target_v = 0.0
                target_omega = 0.0
            elif self.state == "navegar":
                qr_id = self.check_near_qr(pos)
                if qr_id is not None:
                    self._process_qr(qr_id, pos, obs)
                    target_v = 0.0
                    target_omega = 0.0
                else:
                    if self.route_resumed_after_alert and not self.is_near_start(pos):
                        self.left_start_after_resume = True

                    if self._can_complete_mission(pos) and not getattr(self, 'will_complete', False):
                        print("  🏁 Misión completada. Llegada a Base.")
                        self.wait_timer = 3.0
                        self.will_complete = True
                        target_v = 0.0
                        target_omega = 0.0
                    else:
                        action = self.drive(env, dt)
                        target_v = action[0]
                        target_omega = action[1]

            elif self.state == "retornar_base":
                if self.is_near_start(pos):
                    print("  🏠 Llegada a Base")
                    self.state = "esperar_instrucciones"
                    cfg = self.qr_configs.get("QR-05", {})
                    self.wait_timer = cfg.get("tiempo_detencion", 10)
                    print(f"  ⏳ Esperando {self.wait_timer:.0f}s instrucciones del equipo tecnico...")
                    target_v = 0.0
                    target_omega = 0.0
                else:
                    action = self.drive(env, dt)
                    target_v = action[0]
                    target_omega = action[1]

            elif self.state == "esperar_instrucciones":
                if self.wait_timer > 0:
                    self.wait_timer -= dt
                    target_v = 0.0
                    target_omega = 0.0
                else:
                    print("  ▶ Reanudando ruta original...")
                    self.route_resumed_after_alert = True
                    self.left_start_after_resume = False
                    self.state = "navegar"
                    target_v = 0.0
                    target_omega = 0.0

            elif self.state == "completar":
                self.done = True
                target_v = 0.0
                target_omega = 0.0

        # Rampa de desaceleración (suavizado)
        self.current_v = 0.8 * self.current_v + 0.2 * target_v
        self.current_omega = 0.8 * self.current_omega + 0.2 * target_omega

        return np.array([self.current_v, self.current_omega])

    def _advance_qr_queue(self):
        self.qr_index += 1

    def _set_post_classification_route(self):
        target = "QR-07" if self.clasificacion == "aprobado" else "QR-08"
        print(f"  🎯 Desviando a {target} ({self.clasificacion})")
        self.qr_queue = [target, "QR-00"]
        self.qr_index = 0

    def _process_qr(self, qr_id, pos, obs):
        cfg = self.qr_configs.get(qr_id, {})
        nombre = cfg.get("nombre_punto", qr_id)
        print(f"  📍 QR detectado: {qr_id} - {nombre}")
        self.last_detected_qr = qr_id
        self.visited_qrs.add(qr_id)

        tiempo = cfg.get("tiempo_detencion", 3)
        repeticiones = cfg.get("repeticiones", 1)

        if qr_id == "QR-05":
            print("  🚨 ¡ALERTA SANITARIA! Piojo de mar detectado")
            print("  🔴 Interrumpiendo ruta inmediatamente")
            self.simulate_lights(cfg)
            self.state = "retornar_base"
            self._advance_qr_queue()
            return

        if qr_id in ("QR-01", "QR-02", "QR-03"):
            print(f"  ✅ {nombre} - Confirmando trazabilidad")
            self.simulate_lights(cfg)
            self.wait_timer = tiempo
            self._advance_qr_queue()
            return

        if qr_id == "QR-06":
            print("  🔬 Zona de Inspeccion - Clasificando objeto con vision artificial...")
            self.clasificacion = classify_object(obs)
            print(f"  📊 Resultado clasificacion: {self.clasificacion.upper()}")
            self.wait_timer = tiempo
            self._set_post_classification_route()
            return

        if qr_id in ("QR-07", "QR-08"):
            resultado = cfg.get("resultado", qr_id)
            print(f"  📦 Objeto desviado a Zona {resultado.upper()}")
            self.wait_timer = tiempo
            self._advance_qr_queue()
            return

        if qr_id == "QR-00":
            print("  🏁 Registrando clasificacion en Base")
            if self.clasificacion:
                print(f"  📝 Resultado registrado: {self.clasificacion.upper()}")
            self.wait_timer = tiempo
            self._advance_qr_queue()
            if self.challenge_id == 3:
                self.will_complete = True
            return

        self.wait_timer = tiempo
        self._advance_qr_queue()

    def on_collision(self):
        self.collisions += 1

    def is_done(self):
        return self.done


def run_challenge(challenge_num):
    challenges_data = load_challenges()
    challenges = challenges_data.get("challenges", [])

    challenge_cfg = None
    for c in challenges:
        if c.get("id") == challenge_num:
            challenge_cfg = c
            break

    if challenge_cfg is None:
        print(f"\n  ❌ Desafio {challenge_num} no encontrado")
        return

    nombre = challenge_cfg.get("nombre", f"Desafio {challenge_num}")
    tipo = challenge_cfg.get("tipo", "")
    qrs = challenge_cfg.get("qrs", [])

    print(f"\n  ========================================")
    print(f"  🏁 DESAFIO {challenge_num}: {nombre}")
    print(f"  ========================================")
    print(f"  Tipo : {tipo.upper()}")
    print(f"  QRs  : {', '.join(qrs)}")
    print(f"  {challenge_cfg.get('descripcion', '')}")
    print()

    env = DuckietownEnv(
        seed=1,
        map_name=MAP_NAME,
        draw_curve=False,
        draw_bbox=False,
        domain_rand=False,
        frame_skip=1,
        distortion=False,
        max_steps=999999, # Evita reset automático por timeout
    )
    env.reset()
    env.render()

    solver = ChallengeSolver(challenge_cfg)
    solver.challenge_start_time = time.time()

    cam_mode = [0]
    key_handler = key.KeyStateHandler()
    env.unwrapped.window.push_handlers(key_handler)

    running = [True]

    @env.unwrapped.window.event
    def on_close():
        running[0] = False
        pyglet.app.exit()

    @env.unwrapped.window.event
    def on_key_press(symbol, modifiers):
        if symbol == key.ESCAPE:
            running[0] = False
            pyglet.app.exit()
        elif symbol == key.C:
            cam_mode[0] = 1 - cam_mode[0]
            print(f"  Camara: {CAM_NAMES[cam_mode[0]]}")
        elif symbol == key.R:
            env.reset()
            solver.reset()
            solver.challenge_start_time = time.time()
            print("  Reset")

    def safe_update(dt):
        nonlocal last_obs
        if not running[0]:
            return

        if cam_mode[0] == 1:
            angle = env.unwrapped.cur_angle
            dx = np.cos(angle)
            dz = -np.sin(angle)
            env.unwrapped.cam_offset = np.array([-dx * 0.6, 0.0, -dz * 0.6])
            env.unwrapped.cam_height = 0.35
            env.unwrapped.cam_angle = [25.0, 0.0, 0.0]
        else:
            env.unwrapped.cam_offset = np.array([0.0, 0.0, 0.0])
            env.unwrapped.cam_height = 0.108
            env.unwrapped.cam_angle = [15.0, 0.0, 0.0]

        try:
            if solver.is_done():
                print_detailed_result(challenge_num, nombre, solver, qrs)
                print("  🏁 Simulación finalizada. Cerrando ventana...")
                pyglet.app.exit()
                return

            action = solver.get_action(last_obs, env, dt)
            obs, reward, done, info = env.step(action)
            last_obs = obs
            solver.total_reward += reward

            if info.get("collision", False):
                solver.on_collision()

            if done:
                # Comentado para evitar "teletransportes" accidentales por límite de pasos o colisiones menores
                # env.reset()
                pass

            env.render()
        except Exception as e:
            print(f"\n  ⚠ Error: {e}")
            running[0] = False
            pyglet.app.exit()

    last_obs = env.reset()
    env.render()

    pyglet.clock.schedule_interval(safe_update, 1.0 / env.unwrapped.frame_rate)
    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        print("\n  ⚠ Interrumpido")
    finally:
        pyglet.clock.unschedule(safe_update)
        try:
            env.close()
        except Exception:
            pass


if __name__ == "__main__":
    num = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    run_challenge(num)
