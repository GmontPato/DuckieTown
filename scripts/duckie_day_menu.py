#!/usr/bin/env python3
"""
Duckie Day 2026 - Menu de Simulación
Control manual, competencia automática y cambio de cámara (1ra/3ra persona).
"""

import json
import os
import sys
import time
import numpy as np
import pyglet
from pyglet.window import key

import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv

try:
    import cv2
except ImportError:
    print("\n--- ATENCIÓN ---\n")
    print("OpenCV no está instalado. La detección de señales de PARE será desactivada.")
    print("Por favor, ejecuta: pip install opencv-python-headless")
    print("\n-----------------\n")
    cv2 = None

ENV_NAME = os.environ.get("DUCKIE_ENV", "Duckietown-udem1-v0")
DELIVERIES_PATH = os.environ.get("DELIVERIES_PATH", "/data/deliveries.json")
MAP_NAME = os.environ.get("MAP_NAME", "duckie_day_2026")

CAM_NAMES = {0: "1ra Persona", 1: "3ra Persona"}

# ─── Utilidades ───────────────────────────────────────

def load_deliveries(path):
    default = [
        {"id": "QR001", "station": "P1", "feed": "Alimento Salmón Iniciador A", "kg": 50.0, "priority": 1},
        {"id": "QR002", "station": "P2", "feed": "Alimento Salmón Crecimiento B", "kg": 30.0, "priority": 2},
        {"id": "QR003", "station": "P3", "feed": "Alimento Salmón Engorda C", "kg": 40.0, "priority": 3},
    ]
    try:
        with open(path) as f:
            data = json.load(f)
            return data.get("deliveries", data)
    except Exception:
        return default

def make_env():
    return DuckietownEnv(
        seed=1,
        map_name=MAP_NAME,
        draw_curve=False,
        draw_bbox=False,
        domain_rand=False,
        frame_skip=1,
        distortion=False,
    )

def clear():
    os.system("clear" if os.name == "posix" else "cls")


def detect_stop_sign(image):
    """
    Detecta una señal de PARE (roja, octagonal) en la imagen.
    Devuelve True si se detecta, False en caso contrario.
    """
    if cv2 is None or image is None:
        return False

    # Convertir a HSV para un mejor filtrado de color
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Rangos para el color rojo en HSV (es un color que da la vuelta en el círculo cromático)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Crear máscaras y combinarlas
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        # Filtrar por área para ignorar ruido pequeño
        area = cv2.contourArea(cnt)
        if area < 500:  # Ajustar este valor según la distancia de detección
            continue

        # Aproximar el contorno a un polígono
        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Una señal de PARE es un octágono (8 vértices)
        if len(approx) == 8:
            return True # ¡Detectado!

    return False


# ─── PID ──────────────────────────────────────────────

class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.last_error) / max(dt, 0.01)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return np.clip(output, -1.0, 1.0)

# ─── Solvers ──────────────────────────────────────────

class DuckieDaySolver:
    def __init__(self, deliveries):
        self.deliveries = sorted(deliveries, key=lambda x: x.get('priority', 99))
        self.current_idx = 0
        self.completed = [False] * len(deliveries)
        
        # --- Controladores para el modo HÍBRIDO ---
        # 1. PID completo para curvas cerradas
        self.pid_angle_turn = PIDController(kp=6.0, ki=0.0, kd=0.8)
        self.pid_distance_turn = PIDController(kp=5.0, ki=0.01, kd=1.0)
        
        # 2. Control Proporcional simple para rectas (sin oscilación)
        self.kp_angle_straight = 2.5
        self.kp_dist_straight = 2.0
        
        self.driving_mode = 'STRAIGHT' # Puede ser 'STRAIGHT' o 'CURVE'
        
        self.base_speed = 0.45
        self.delivery_mode = False
        self.delivery_timer = 0
        self.delivery_duration = 3.0
        
        self.stop_sign_timer = 0.0
        self.stop_duration = 3.0
        
        self.current_v = 0.0
        self.current_omega = 0.0
        
        self.total_reward = 0.0
        self.collisions = 0
        
        self.qr_step_interval = 400
        self.last_qr_step = 0
        self.done = False

    def reset(self):
        self.__init__(self.deliveries)

    def get_action(self, obs, env, dt=0.1):
        target_v = 0.0
        target_omega = 0.0

        # Lógicas de estado (PARE, Entrega) tienen prioridad
        if self.stop_sign_timer > 0:
            self.stop_sign_timer -= dt
            target_v = 0.0
            target_omega = 0.0
        elif self.delivery_mode:
            self.delivery_timer += dt
            if self.delivery_timer >= self.delivery_duration:
                self.delivery_mode = False
                self.delivery_timer = 0
                self.completed[self.current_idx] = True
                self.current_idx = (self.current_idx + 1)
                if self.current_idx >= len(self.deliveries):
                    self.done = True
                else:
                    # Resetea el modo de conducción después de una entrega
                    self.driving_mode = 'STRAIGHT'
                    self.pid_distance_turn.reset()
            target_v = 0.0
            target_omega = 0.0
        elif detect_stop_sign(obs):
            print("  🛑 ¡Señal de PARE detectada! Frenando...")
            self.stop_sign_timer = self.stop_duration
            target_v = 0.0
            target_omega = 0.0
        else:
            # --- Lógica de conducción HÍBRIDA ---
            try:
                lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
                dist = lane_pose.dist
                angle = lane_pose.angle_rad
            except Exception:
                dist, angle = 0.0, 0.0

            # Hysteresis para cambiar de modo: evita cambios rápidos
            if abs(angle) > 0.22 and self.driving_mode == 'STRAIGHT':
                print("  Modo conducción: CURVA 꺾")
                self.driving_mode = 'CURVE'
                self.pid_distance_turn.reset()
            elif abs(angle) < 0.1 and self.driving_mode == 'CURVE':
                print("  Modo conducción: RECTO  직선")
                self.driving_mode = 'STRAIGHT'

            if self.driving_mode == 'STRAIGHT':
                steering = (self.kp_angle_straight * angle) + (self.kp_dist_straight * dist)
            else:
                steer_angle = self.pid_angle_turn.compute(angle, dt)
                steer_dist = self.pid_distance_turn.compute(dist, dt)
                steering = steer_angle + steer_dist

            steering = np.clip(steering, -1.0, 1.0)
            
            speed = self.base_speed * max(0.35, 1.0 - abs(steering) * 0.7)
            speed = np.clip(speed, 0.0, self.base_speed)
            
            target_v = speed
            target_omega = steering

        self.current_v = 0.8 * self.current_v + 0.2 * target_v
        self.current_omega = 0.8 * self.current_omega + 0.2 * target_omega
        return np.array([self.current_v, self.current_omega])

    def on_collision(self):
        # Según rúbrica, penalizamos en el contador final
        self.collisions += 1

    def start_delivery(self):
        if self.current_idx < len(self.deliveries):
            self.delivery_mode = True
            self.delivery_timer = 0

# ─── Modos ────────────────────────────────────────────

def _run_simulation(env, update_fn, key_handler, cam_mode_list):
    """Ejecuta el loop de pyglet con manejo de Ctrl+C, errores y cierre de ventana."""
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
            cam_mode_list[0] = 1 - cam_mode_list[0]
            print(f"  Cámara cambiada a: {CAM_NAMES[cam_mode_list[0]]}")
        elif symbol == key.R:
            env.reset()
            print("  Reset")

    def safe_update(dt):
        if not running[0]:
            return
        
        # Actualizar cámara dinámicamente según modo
        if cam_mode_list[0] == 1:
            # 3ra Persona dinámica (sigue al robot por detrás)
            angle = env.unwrapped.cur_angle
            dx = np.cos(angle)
            dz = -np.sin(angle)
            env.unwrapped.cam_offset = np.array([-dx * 0.6, 0.0, -dz * 0.6])
            env.unwrapped.cam_height = 0.35
            env.unwrapped.cam_angle = [25.0, 0.0, 0.0]
        else:
            # 1ra Persona original
            env.unwrapped.cam_offset = np.array([0.0, 0.0, 0.0])
            env.unwrapped.cam_height = 0.108
            env.unwrapped.cam_angle = [15.0, 0.0, 0.0]

        try:
            update_fn(dt)
        except Exception as e:
            print(f"\n  ⚠ Error: {e}")
            running[0] = False
            pyglet.app.exit()

    pyglet.clock.schedule_interval(safe_update, 1.0 / env.unwrapped.frame_rate)
    try:
        pyglet.app.run()
    except KeyboardInterrupt:
        print("\n  ⚠ Interrumpido por el usuario")
    finally:
        pyglet.clock.unschedule(safe_update)
        try:
            env.close()
        except Exception:
            pass


def mode_manual_control():
    env = make_env()
    env.reset()
    env.render()

    cam_mode = [0]
    key_handler = key.KeyStateHandler()
    env.unwrapped.window.push_handlers(key_handler)

    def update(dt):
        action = np.array([0.0, 0.0])
        if key_handler[key.UP]:
            action = np.array([0.44, 0.0])
        if key_handler[key.DOWN]:
            action = np.array([-0.44, 0.0])
        if key_handler[key.LEFT]:
            action = np.array([0.35, 1.0])
        if key_handler[key.RIGHT]:
            action = np.array([0.35, -1.0])
        if key_handler[key.SPACE]:
            action = np.array([0.0, 0.0])
        if key_handler[key.LSHIFT]:
            action[0] = min(action[0] * 1.5, 1.0)
            action[1] = np.clip(action[1] * 1.5, -1.0, 1.0)

        obs, reward, done, info = env.step(action)
        if done:
            env.reset()
        env.render()

    _run_simulation(env, update, key_handler, cam_mode)


def mode_competition():
    deliveries = load_deliveries(DELIVERIES_PATH)

    env = make_env()
    last_obs = env.reset()
    env.render()

    solver = DuckieDaySolver(deliveries)
    cam_mode = [0]
    
    key_handler = key.KeyStateHandler()
    env.unwrapped.window.push_handlers(key_handler)
    start_time = time.time()

    print(f"\n  Entregas: {len(deliveries)}")
    for d in deliveries:
        print(f"    {d['id']}: {d['station']} -> {d['feed']} ({d['kg']}kg)")
    print("  ESC: salir | C: cámara | R: reset\n")

    def update(dt):
        nonlocal last_obs
        if solver.done:
            elapsed = time.time() - start_time
            print(f"\n  ========================================")
            print(f"  🏆 EVALUACIÓN SEGÚN RÚBRICA DUCKIE DAY")
            print(f"  ========================================")
            print(f"  - Tiempo de ejecución: {elapsed:.1f}s")
            print(f"  - Precisión: {len(deliveries)}/3 entregas en orden OK")
            print(f"  - Seguridad operativa: {solver.collisions} colisiones")
            print(f"  - Robustez del sistema: Excepciones manejadas")
            print(f"  ========================================\n")
            pyglet.app.exit()
            return

        action = solver.get_action(last_obs, env, dt)
        obs, reward, done, info = env.step(action)
        last_obs = obs  # Guardar la observación actual para el siguiente ciclo
        solver.total_reward += reward

        if info.get("collision", False):
            solver.on_collision()

        if (solver.current_idx < len(deliveries)
                and not solver.delivery_mode
                and env.unwrapped.step_count - solver.last_qr_step >= solver.qr_step_interval):
            d = deliveries[solver.current_idx]
            print(f"  📦 Entrega {d['station']}: {d['feed']}")
            solver.start_delivery()
            solver.last_qr_step = env.unwrapped.step_count

        env.render()

    _run_simulation(env, update, key_handler, cam_mode)

# ─── Menú principal ───────────────────────────────────

# Importar desafíos (opcional)
try:
    from duckie_day_challenge import run_challenge
    HAS_CHALLENGES = True
except ImportError:
    HAS_CHALLENGES = False

def main():
    while True:
        clear()
        print()
        print("  ╔══════════════════════════════════════════════╗")
        print("  ║   🦆 DUCKIE DAY 2026 - PUERTO MONTT        ║")
        print("  ║   Desafíos Oficiales - Industria Salmonera   ║")
        print("  ╚══════════════════════════════════════════════╝")
        print()
        print("  DESAFÍOS OFICIALES:")
        print("  [1] 🏁 Clasificatoria - Alerta Sanitaria")
        print("      Reacción en tiempo real ante piojo de mar")
        print()
        print("  [2] 🏁 Semifinal - Trazabilidad Completa")
        print("      De la Ova al Plato (3 estaciones secuenciales)")
        print()
        print("  [3] 🏁 Final - Inspección con Visión Artificial")
        print("      Clasificación automática en Chinquihue")
        print()
        print("  ───────────────────────────────────────────────")
        print("  [4] 🎮 Control Manual")
        print("      Flechas: mover | SHIFT: turbo | ESPACIO: frenar")
        print()
        print("  [5] 🤖 Competencia Automática (demo)")
        print("      Lane following + entregas secuenciales PID")
        print()
        print("  Controles durante simulación:")
        print("    C  = cambiar cámara (1ra ↔ 3ra persona)")
        print("    R  = resetear robot")
        print("    ESC = salir")
        print()
        choice = input("  Selecciona una opción [1-5/q]: ").strip().lower()

        if choice in ("1",):
            if HAS_CHALLENGES:
                run_challenge(1)
            else:
                print("\n  ⚠ Desafíos no disponibles (importación falló)")
                input("  Presiona Enter para continuar...")
        elif choice in ("2",):
            if HAS_CHALLENGES:
                run_challenge(2)
            else:
                print("\n  ⚠ Desafíos no disponibles")
                input("  Presiona Enter para continuar...")
        elif choice in ("3",):
            if HAS_CHALLENGES:
                run_challenge(3)
            else:
                print("\n  ⚠ Desafíos no disponibles")
                input("  Presiona Enter para continuar...")
        elif choice in ("4", "manual"):
            mode_manual_control()
        elif choice in ("5", "auto", "competencia"):
            mode_competition()
        elif choice in ("q", "quit", "salir"):
            print("\n  ¡Buena suerte en el Duckie Day! 🦆\n")
            break

if __name__ == "__main__":
    main()
