# ==========================================
# 1. START SYMULACJI (Musi być na samej górze)
# ==========================================
from omni.isaac.kit import SimulationApp
# headless=False -> uruchamia okno z grafiką
simulation_app = SimulationApp({"headless": False})

# ==========================================
# 2. IMPORTY
# ==========================================
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.urdf import _urdf
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import os
import sys

# ==========================================
# 3. KONFIGURACJA ŚCIEŻEK (AUTOMATYCZNA)
# ==========================================

# Pobieramy lokalizację tego skryptu (.../boxygo_description/isaac_scripts)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Wychodzimy piętro wyżej -> jesteśmy w 'boxygo_description'
PACKAGE_ROOT = os.path.dirname(SCRIPT_DIR)

# Wychodzimy jeszcze wyżej -> jesteśmy w folderze 'src' (lub 'install/share')
# To jest folder, w którym Isaac będzie szukał pakietów ROS (package://)
ROS_PACKAGE_PATH = os.path.dirname(PACKAGE_ROOT)

# Ścieżka do konkretnego pliku URDF
# Wybrałem 'luksusowy_isaac.urdf' bo widzę go na zdjęciu w folderze urdf
# Jeśli chcesz użyć innego, zmień nazwę pliku poniżej:
URDF_FILENAME = "luksusowy_isaac.urdf" 
URDF_PATH = os.path.join(PACKAGE_ROOT, "urdf", URDF_FILENAME)

# ==========================================
# 4. GŁÓWNA FUNKCJA
# ==========================================

def main():
    # Inicjalizacja świata
    world = World()
    world.scene.add_default_ground_plane()

    # Weryfikacja czy plik istnieje
    if not os.path.exists(URDF_PATH):
        print(f"\n[BŁĄD] Nie znaleziono pliku: {URDF_PATH}")
        print(f"Upewnij się, że wykonałeś konwersję XACRO -> URDF w Dockerze!")
        simulation_app.close()
        return

    # --- Konfiguracja Importera URDF ---
    urdf_interface = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()
    
    # KLUCZOWE: Dodajemy ścieżkę nadrzędną do wyszukiwania zasobów
    # Dzięki temu package://boxygo_description/... zadziała poprawnie
    import_config.set_search_paths([ROS_PACKAGE_PATH])
    
    # Ustawienia fizyki i importu
    import_config.merge_fixed_joints = False
    import_config.fix_base = False       # Ustaw True, jeśli robot ma być przyklejony
    import_config.make_default_prim = True
    import_config.self_collision = False # Często wyłącza się, by uniknąć "drgawek"
    import_config.create_physics_scene = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

    # --- Import Robota ---
    print(f"Importowanie: {URDF_PATH}")
    # Wstawiamy robota do ścieżki /World/luksusowy
    prim_path = urdf_interface.import_urdf(URDF_PATH, "/World/luksusowy", import_config)

    if prim_path:
        # Rejestracja robota w silniku fizyki Isaaca
        # Zmieniamy pozycję Z na 0.3, żeby nie wpadł pod podłogę przy starcie
        robot = Robot(prim_path=prim_path, name="luksusowy_robot")
        robot.set_world_pose(position=np.array([0.0, 0.0, 0.3]))
        world.scene.add(robot)
        print("-> SUKCES: Robot zaimportowany!")
    else:
        print("-> BŁĄD: Nie udało się zaimportować URDF.")

    # Reset fizyki (niezbędne po dodaniu obiektów)
    world.reset()

    # Ustawienie kamery (patrzy na robota)
    set_camera_view(eye=np.array([2.5, 2.5, 1.5]), target=np.array([0, 0, 0]))

    # --- Pętla Symulacji ---
    while simulation_app.is_running():
        world.step(render=True)
        
        # Tu możesz dopisać kod sterowania, np.:
        # robot.set_joint_velocities(...)

    simulation_app.close()

if __name__ == "__main__":
    main()