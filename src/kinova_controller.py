import redis
import signal
import sys
import time
from typing import Union
import numpy as np
import argparse

from ik_solver import IKSolver
from config_reader import ConfigReader
from timing_utils import HighPrecisionTimer, format_timing_stats

def decode_redis_str(s: Union[str, bytes]) -> np.ndarray:
    """Decodes a matrix encoded as a string into a numpy array."""
    if isinstance(s, bytes):
        s = s.decode("utf8")
    s = s.strip()
    tokens = [list(map(float, row.strip().split())) for row in s.split(";")]
    return np.array(tokens).squeeze()


def encode_redis_str(A: np.ndarray) -> str:
    """Encodes a numpy array to a string."""
    if len(A.shape) == 1:
        return " ".join(map(str, A.tolist()))
    return "; ".join(" ".join(map(str, row)) for row in A.tolist())


def main(args):
    # Load configuration
    try:
        config = ConfigReader(args.config)
    except Exception as e:
        print(f"Error loading configuration: {e}")
        sys.exit(1)
    
    # Validate robot number
    if not config.validate_robot_number(args.robot):
        print(f"Error: Robot {args.robot} is not configured in {args.config}")
        print(f"Available robots: {config.get_all_robot_numbers()}")
        sys.exit(1)
    
    # Get robot and Redis configuration
    robot_config = config.get_robot_config(args.robot)
    redis_config = config.get_redis_config()
    central_redis_host = config.get_central_redis_host()
    timing_config = config.get_timing_config()
    
    # Initialize redis keys
    KEY_KINOVA_Q = f"kinova::bot{args.robot}::q"
    KEY_KINOVA_Q_DES = f"kinova::bot{args.robot}::q_des"
    KEY_KINOVA_EE_POS = f"kinova::bot{args.robot}::ee_pos"
    KEY_KINOVA_EE_QUAT_WXYZ = f"kinova::bot{args.robot}::ee_quat_wxyz"
    KEY_KINOVA_EE_POS_DES = f"kinova::bot{args.robot}::ee_pos_des"
    KEY_KINOVA_EE_QUAT_WXYZ_DES = f"kinova::bot{args.robot}::ee_quat_wxyz_des"
    KEY_KINOVA_GRIPPER_POS = f"kinova::bot{args.robot}::gripper_position"
    KEY_KINOVA_GRIPPER_POS_DES = f"kinova::bot{args.robot}::gripper_position_des"
    KEY_KINOVA_STATUS = f"kinova::bot{args.robot}::status"

    # Initialize Redis clients
    redis_client_local = redis.Redis(
        host=robot_config['redis_host'], 
        port=redis_config['port'], 
        db=0, 
        password=redis_config['password']
    )
    redis_client = redis.Redis(
        host=central_redis_host, 
        port=redis_config['port'], 
        db=0, 
        password=redis_config['password']
    )
    ee_pos_str = redis_client_local.get(KEY_KINOVA_EE_POS)
    ee_quat_wxyz_str = redis_client_local.get(KEY_KINOVA_EE_QUAT_WXYZ)
    gripper_pos_str = redis_client_local.get(KEY_KINOVA_GRIPPER_POS)
    redis_client_local.set(KEY_KINOVA_EE_POS_DES, ee_pos_str)
    redis_client_local.set(KEY_KINOVA_EE_QUAT_WXYZ_DES, ee_quat_wxyz_str)
    redis_client_local.set(KEY_KINOVA_GRIPPER_POS_DES, gripper_pos_str)
    redis_client.set(KEY_KINOVA_EE_POS_DES, ee_pos_str)
    redis_client.set(KEY_KINOVA_EE_QUAT_WXYZ_DES, ee_quat_wxyz_str)
    redis_client.set(KEY_KINOVA_GRIPPER_POS_DES, "0")

    def signal_handler(sig, frame):
        """Handle Ctrl-C by setting redis key status to 0 before exiting."""
        print("\nGracefully shutting down...")
        redis_client.set(KEY_KINOVA_STATUS, '0')
        redis_client_local.set(KEY_KINOVA_STATUS, '0')
        sys.exit(0)

    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)

    ik_solver = IKSolver(ee_offset=0.148)

    # Initialize high-precision timer
    controller_frequency = timing_config.get('controller_frequency', 100.0)
    stats_print_interval = timing_config.get('stats_print_interval', 5.0)
    timer = HighPrecisionTimer(target_frequency=controller_frequency)
    
    last_stats_print = time.time()

    while True:
        # Check if it's time to run the next iteration
        timing_error = timer.update()
        if timing_error is None:
            # We're ahead of schedule, sleep until next target time
            timer.sleep_until_next()
            continue
        
        # Print timing statistics periodically
        current_time = time.time()
        if current_time - last_stats_print > stats_print_interval:
            stats = timer.get_stats()
            print(format_timing_stats(stats))
            last_stats_print = current_time

        # Get the ee command
        ee_pos_des_str = redis_client.get(KEY_KINOVA_EE_POS_DES)
        ee_quat_wxyz_des_str = redis_client.get(KEY_KINOVA_EE_QUAT_WXYZ_DES)
        gripper_pos_des_str = redis_client.get(KEY_KINOVA_GRIPPER_POS_DES)
        ee_pos_des = decode_redis_str(ee_pos_des_str)
        ee_quat_wxyz_des = decode_redis_str(ee_quat_wxyz_des_str)
        q_s_str = redis_client_local.get(KEY_KINOVA_Q)
        q_s = decode_redis_str(q_s_str) * np.pi / 180.0

        # Send joint command to local redis
        q_command = ik_solver.solve(ee_pos_des, ee_quat_wxyz_des, q_s)
        q_command = q_command * 180.0 / np.pi
        redis_client_local.set(KEY_KINOVA_Q_DES, encode_redis_str(q_command))
        redis_client_local.set(KEY_KINOVA_GRIPPER_POS_DES, gripper_pos_des_str)

        # Update remote redis
        ee_pos_str = redis_client_local.get(KEY_KINOVA_EE_POS)
        ee_quat_wxyz_str = redis_client_local.get(KEY_KINOVA_EE_QUAT_WXYZ)
        gripper_pos_str = redis_client_local.get(KEY_KINOVA_GRIPPER_POS)       
        redis_client.set(KEY_KINOVA_EE_POS, ee_pos_str)
        redis_client.set(KEY_KINOVA_EE_QUAT_WXYZ, ee_quat_wxyz_str)
        redis_client.set(KEY_KINOVA_GRIPPER_POS, gripper_pos_str)
        redis_client.set(KEY_KINOVA_Q, q_s_str)

        # Check kinova status
        status_str = redis_client_local.get(KEY_KINOVA_STATUS)
        status = int(decode_redis_str(status_str))
        if status == 0:
            print("Shutting down...")
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=int, default=1, help="Robot number")
    parser.add_argument("--config", type=str, default="config/robot_config.json", 
                       help="Path to configuration file")
    args = parser.parse_args()
    main(args)