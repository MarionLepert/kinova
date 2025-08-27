import json
import os
from typing import Dict, Any


class ConfigReader:
    """Configuration reader for robot settings."""
    
    def __init__(self, config_path: str = "config/robot_config.json"):
        """
        Initialize the configuration reader.
        
        Args:
            config_path: Path to the configuration JSON file
        """
        self.config_path = config_path
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """
        Load configuration from JSON file.
        
        Returns:
            Dictionary containing the configuration
            
        Raises:
            FileNotFoundError: If config file doesn't exist
            json.JSONDecodeError: If config file is invalid JSON
            KeyError: If required configuration keys are missing
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
        except json.JSONDecodeError as e:
            raise json.JSONDecodeError(f"Invalid JSON in config file {self.config_path}: {e}")
        
        # Validate required keys
        required_keys = ['robots', 'redis', 'central_redis_host']
        for key in required_keys:
            if key not in config:
                raise KeyError(f"Missing required key '{key}' in configuration file")
        
        # Validate timing section if present
        if 'timing' in config:
            timing_required_keys = ['controller_frequency', 'timing_window_size', 'stats_print_interval']
            for key in timing_required_keys:
                if key not in config['timing']:
                    raise KeyError(f"Missing required key '{key}' in timing configuration")
        
        # Validate robots section
        if not isinstance(config['robots'], dict):
            raise ValueError("'robots' section must be a dictionary")
        
        # Validate redis section
        redis_required_keys = ['port', 'password']
        for key in redis_required_keys:
            if key not in config['redis']:
                raise KeyError(f"Missing required key '{key}' in redis configuration")
        
        return config
    
    def get_robot_config(self, robot_number: int) -> Dict[str, str]:
        """
        Get configuration for a specific robot.
        
        Args:
            robot_number: Robot number (1, 2, etc.)
            
        Returns:
            Dictionary containing robot configuration (ip_address, redis_host)
            
        Raises:
            KeyError: If robot configuration is not found
        """
        robot_key = str(robot_number)
        if robot_key not in self.config['robots']:
            raise KeyError(f"Robot configuration not found for robot number: {robot_number}")
        
        robot_config = self.config['robots'][robot_key]
        
        # Validate robot configuration
        required_keys = ['ip_address', 'redis_host']
        for key in required_keys:
            if key not in robot_config:
                raise KeyError(f"Missing required key '{key}' in robot {robot_number} configuration")
        
        return robot_config
    
    def get_redis_config(self) -> Dict[str, Any]:
        """
        Get Redis configuration.
        
        Returns:
            Dictionary containing Redis configuration (port, password)
        """
        return self.config['redis']
    
    def get_central_redis_host(self) -> str:
        """
        Get the central Redis host address.
        
        Returns:
            Central Redis host address
        """
        return self.config['central_redis_host']
    
    def get_timing_config(self) -> dict:
        """
        Get timing configuration.
        
        Returns:
            Dictionary containing timing configuration
        """
        return self.config.get('timing', {})
    
    def get_all_robot_numbers(self) -> list:
        """
        Get list of all configured robot numbers.
        
        Returns:
            List of robot numbers as integers
        """
        return [int(robot_id) for robot_id in self.config['robots'].keys()]
    
    def validate_robot_number(self, robot_number: int) -> bool:
        """
        Check if a robot number is configured.
        
        Args:
            robot_number: Robot number to validate
            
        Returns:
            True if robot is configured, False otherwise
        """
        return str(robot_number) in self.config['robots']


def load_config(config_path: str = "config/robot_config.json") -> ConfigReader:
    """
    Convenience function to load configuration.
    
    Args:
        config_path: Path to the configuration JSON file
        
    Returns:
        ConfigReader instance
    """
    return ConfigReader(config_path) 