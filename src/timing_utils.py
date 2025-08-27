#!/usr/bin/env python3
"""
High-precision timing utilities for robot control loops.
"""

import time
from typing import List, Optional
from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass
class TimingStats:
    """Timing statistics for control loops."""
    target_frequency: float
    actual_frequency: float
    avg_timing_error_us: float
    max_timing_error_us: float
    min_timing_error_us: float
    jitter_us: float
    total_iterations: int
    elapsed_time_s: float


class HighPrecisionTimer:
    """
    High-precision timer for control loops with statistics tracking.
    
    This timer provides microsecond precision timing and tracks timing errors,
    jitter, and frequency statistics for real-time control applications.
    """
    
    def __init__(self, target_frequency: float = 100.0, window_size: int = 1000):
        """
        Initialize the timer.
        
        Args:
            target_frequency: Target frequency in Hz
            window_size: Number of samples to keep for statistics
        """
        self.target_frequency = target_frequency
        self.target_period_s = 1.0 / target_frequency
        self.window_size = window_size
        
        # Timing state
        self.start_time = time.perf_counter()
        self.last_iteration_time = self.start_time
        self.loop_counter = 0
        
        # Statistics tracking
        self.timing_errors = deque(maxlen=window_size)
        self.loop_times = deque(maxlen=window_size)
        
        # Performance tracking
        self.max_timing_error_us = 0
        self.min_timing_error_us = float('inf')
        
    def get_current_time_s(self) -> float:
        """Get current time in seconds."""
        return time.perf_counter()
    
    def get_elapsed_time_s(self) -> float:
        """Get elapsed time since timer creation in seconds."""
        return time.perf_counter() - self.start_time
    
    def should_run(self) -> bool:
        """
        Check if it's time to run the next iteration.
        
        Returns:
            True if the target period has elapsed, False otherwise
        """
        current_time = self.get_current_time_s()
        expected_time = self.start_time + (self.loop_counter * self.target_period_s)
        return current_time >= expected_time
    
    def update(self) -> Optional[float]:
        """
        Update timer state and return timing error.
        
        Returns:
            Timing error in seconds, or None if not ready to run
        """
        if not self.should_run():
            return None
        
        current_time = self.get_current_time_s()
        expected_time = self.start_time + (self.loop_counter * self.target_period_s)
        
        # Calculate timing error in seconds
        timing_error_s = current_time - expected_time
        
        # Convert to microseconds for statistics
        timing_error_us = timing_error_s * 1_000_000
        
        # Update statistics
        self.timing_errors.append(timing_error_us)
        
        if timing_error_us > self.max_timing_error_us:
            self.max_timing_error_us = timing_error_us
        if timing_error_us < self.min_timing_error_us:
            self.min_timing_error_us = timing_error_us
        
        # Calculate loop time
        loop_time = current_time - self.last_iteration_time
        self.loop_times.append(loop_time)
        self.last_iteration_time = current_time
        
        # Update counter
        self.loop_counter += 1
        
        return timing_error_s
    
    def get_stats(self) -> TimingStats:
        """
        Get current timing statistics.
        
        Returns:
            TimingStats object with current statistics
        """
        elapsed_time_s = self.get_elapsed_time_s()
        
        # Calculate actual frequency based on iterations and elapsed time
        actual_frequency = self.loop_counter / elapsed_time_s if elapsed_time_s > 0 else 0
        
        # Calculate timing error statistics
        if self.timing_errors:
            avg_timing_error_us = np.mean(self.timing_errors)
            jitter_us = np.std(self.timing_errors)
        else:
            avg_timing_error_us = 0.0
            jitter_us = 0.0
        
        return TimingStats(
            target_frequency=self.target_frequency,
            actual_frequency=actual_frequency,
            avg_timing_error_us=avg_timing_error_us,
            max_timing_error_us=self.max_timing_error_us,
            min_timing_error_us=self.min_timing_error_us if self.min_timing_error_us != float('inf') else 0.0,
            jitter_us=jitter_us,
            total_iterations=self.loop_counter,
            elapsed_time_s=elapsed_time_s
        )
    
    def reset_stats(self):
        """Reset timing statistics."""
        self.timing_errors.clear()
        self.loop_times.clear()
        self.max_timing_error_us = 0
        self.min_timing_error_us = float('inf')
    
    def sleep_until_next(self):
        """
        Sleep until the next target time to maintain exact frequency.
        """
        current_time = self.get_current_time_s()
        next_target_time = self.start_time + ((self.loop_counter + 1) * self.target_period_s)
        time_until_next = next_target_time - current_time
        
        if time_until_next > 0:
            time.sleep(time_until_next)


class RateLimiter:
    """
    Simple rate limiter for control loops.
    
    This is a simpler alternative to HighPrecisionTimer for cases where
    high precision is not required.
    """
    
    def __init__(self, frequency: float):
        """
        Initialize rate limiter.
        
        Args:
            frequency: Target frequency in Hz
        """
        self.period = 1.0 / frequency
        self.last_time = time.perf_counter()
    
    def sleep(self):
        """Sleep to maintain the target frequency."""
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        
        self.last_time = time.perf_counter()


def format_timing_stats(stats: TimingStats) -> str:
    """
    Format timing statistics for display.
    
    Args:
        stats: TimingStats object
        
    Returns:
        Formatted string with timing information
    """
    return (f"[{stats.elapsed_time_s:.1f}s] "
            f"Frequency: {stats.actual_frequency:.2f} Hz "
            f"(target: {stats.target_frequency:.1f} Hz) | "
            f"Avg error: {stats.avg_timing_error_us:.1f} μs | "
            f"Max error: {stats.max_timing_error_us:.1f} μs | "
            f"Jitter: {stats.jitter_us:.1f} μs")


def create_timer_from_config(config: dict, default_frequency: float = 100.0) -> HighPrecisionTimer:
    """
    Create a timer from configuration.
    
    Args:
        config: Configuration dictionary
        default_frequency: Default frequency if not specified in config
        
    Returns:
        HighPrecisionTimer instance
    """
    frequency = config.get('control_frequency', default_frequency)
    window_size = config.get('timing_window_size', 1000)
    
    return HighPrecisionTimer(target_frequency=frequency, window_size=window_size) 