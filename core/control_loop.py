"""
Core Control Loop for HumanoidOS

This module implements the main control loop that orchestrates all subsystems.
Runs at high frequency (1kHz default) for real-time control.
"""

import time
import logging
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import numpy as np

logger = logging.getLogger(__name__)


class SystemState(Enum):
    """System-wide states"""
    INITIALIZING = "initializing"
    IDLE = "idle"
    STANDING = "standing"
    WALKING = "walking"
    BALANCING = "balancing"
    FALLING = "falling"
    EMERGENCY_STOP = "emergency_stop"
    SHUTDOWN = "shutdown"


@dataclass
class ControlConfig:
    """Configuration for the control loop"""
    frequency_hz: float = 1000.0  # Control loop frequency
    dt: float = field(init=False)  # Time step
    max_cycle_time_ms: float = 5.0  # Maximum allowed cycle time
    enable_safety_checks: bool = True
    enable_telemetry: bool = True
    
    def __post_init__(self):
        self.dt = 1.0 / self.frequency_hz


@dataclass
class SystemMetrics:
    """Real-time system metrics"""
    cycle_time_ms: float = 0.0
    average_cycle_time_ms: float = 0.0
    max_cycle_time_ms: float = 0.0
    control_frequency_hz: float = 0.0
    dropped_cycles: int = 0
    uptime_seconds: float = 0.0
    
    def update(self, cycle_time: float):
        """Update metrics with new cycle time"""
        self.cycle_time_ms = cycle_time * 1000
        if self.cycle_time_ms > self.max_cycle_time_ms:
            self.max_cycle_time_ms = self.cycle_time_ms
        
        # Exponential moving average
        alpha = 0.1
        self.average_cycle_time_ms = (
            alpha * self.cycle_time_ms + 
            (1 - alpha) * self.average_cycle_time_ms
        )
        
        if cycle_time > 0:
            self.control_frequency_hz = 1.0 / cycle_time


class ControlLoop:
    """
    Main control loop for HumanoidOS.
    
    Orchestrates all subsystems and maintains real-time control.
    """
    
    def __init__(self, config: Optional[ControlConfig] = None):
        self.config = config or ControlConfig()
        self.state = SystemState.INITIALIZING
        self.metrics = SystemMetrics()
        
        # Subsystem callbacks
        self._subsystems: Dict[str, Callable] = {}
        self._pre_step_callbacks: list[Callable] = []
        self._post_step_callbacks: list[Callable] = []
        
        # Control flags
        self._running = False
        self._emergency_stop = False
        
        # Timing
        self._start_time = None
        self._last_step_time = None
        
        logger.info(f"Control loop initialized at {self.config.frequency_hz} Hz")
    
    def register_subsystem(self, name: str, callback: Callable):
        """
        Register a subsystem to be called each control cycle.
        
        Args:
            name: Unique name for the subsystem
            callback: Function to call, receives (dt, state) and returns control outputs
        """
        self._subsystems[name] = callback
        logger.info(f"Registered subsystem: {name}")
    
    def add_pre_step_callback(self, callback: Callable):
        """Add callback to run before each control step"""
        self._pre_step_callbacks.append(callback)
    
    def add_post_step_callback(self, callback: Callable):
        """Add callback to run after each control step"""
        self._post_step_callbacks.append(callback)
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        logger.critical("EMERGENCY STOP TRIGGERED")
        self._emergency_stop = True
        self.state = SystemState.EMERGENCY_STOP
    
    def set_state(self, new_state: SystemState):
        """Change system state"""
        old_state = self.state
        self.state = new_state
        logger.info(f"State transition: {old_state.value} -> {new_state.value}")
    
    def step(self) -> Dict[str, Any]:
        """
        Execute one control cycle.
        
        Returns:
            Dictionary of control outputs from all subsystems
        """
        step_start = time.perf_counter()
        
        # Check for emergency stop
        if self._emergency_stop:
            return {"emergency_stop": True}
        
        # Pre-step callbacks (sensor reading, state estimation)
        for callback in self._pre_step_callbacks:
            try:
                callback(self.config.dt, self.state)
            except Exception as e:
                logger.error(f"Pre-step callback error: {e}")
        
        # Execute all subsystems
        outputs = {}
        for name, subsystem in self._subsystems.items():
            try:
                result = subsystem(self.config.dt, self.state)
                if result is not None:
                    outputs[name] = result
            except Exception as e:
                logger.error(f"Subsystem {name} error: {e}")
                if self.config.enable_safety_checks:
                    self.emergency_stop()
        
        # Post-step callbacks (actuation, logging)
        for callback in self._post_step_callbacks:
            try:
                callback(self.config.dt, self.state, outputs)
            except Exception as e:
                logger.error(f"Post-step callback error: {e}")
        
        # Update metrics
        cycle_time = time.perf_counter() - step_start
        self.metrics.update(cycle_time)
        
        # Check for cycle time violations
        if cycle_time > self.config.max_cycle_time_ms / 1000.0:
            logger.warning(
                f"Control cycle took {cycle_time*1000:.2f}ms "
                f"(limit: {self.config.max_cycle_time_ms}ms)"
            )
            self.metrics.dropped_cycles += 1
        
        return outputs
    
    def run(self, duration: Optional[float] = None):
        """
        Run the control loop.
        
        Args:
            duration: Optional duration in seconds. If None, runs indefinitely.
        """
        self._running = True
        self._start_time = time.perf_counter()
        self._last_step_time = self._start_time
        
        self.set_state(SystemState.IDLE)
        logger.info("Control loop started")
        
        try:
            while self._running:
                # Check duration
                if duration is not None:
                    elapsed = time.perf_counter() - self._start_time
                    if elapsed >= duration:
                        break
                
                # Execute control step
                self.step()
                
                # Sleep to maintain frequency
                current_time = time.perf_counter()
                elapsed = current_time - self._last_step_time
                sleep_time = self.config.dt - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                self._last_step_time = time.perf_counter()
                self.metrics.uptime_seconds = self._last_step_time - self._start_time
                
        except KeyboardInterrupt:
            logger.info("Control loop interrupted by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the control loop"""
        self._running = False
        self.set_state(SystemState.SHUTDOWN)
        logger.info("Control loop stopped")
        self._print_metrics()
    
    def _print_metrics(self):
        """Print final metrics"""
        logger.info("=" * 50)
        logger.info("Control Loop Metrics:")
        logger.info(f"  Uptime: {self.metrics.uptime_seconds:.2f}s")
        logger.info(f"  Average cycle time: {self.metrics.average_cycle_time_ms:.2f}ms")
        logger.info(f"  Max cycle time: {self.metrics.max_cycle_time_ms:.2f}ms")
        logger.info(f"  Average frequency: {self.metrics.control_frequency_hz:.1f} Hz")
        logger.info(f"  Dropped cycles: {self.metrics.dropped_cycles}")
        logger.info("=" * 50)


if __name__ == "__main__":
    # Simple test
    logging.basicConfig(level=logging.INFO)
    
    def test_subsystem(dt, state):
        """Test subsystem that just prints"""
        print(f"Tick at {dt}s, state: {state.value}")
        return {"test": "ok"}
    
    config = ControlConfig(frequency_hz=10.0)  # 10 Hz for testing
    loop = ControlLoop(config)
    loop.register_subsystem("test", test_subsystem)
    loop.run(duration=2.0)  # Run for 2 seconds
