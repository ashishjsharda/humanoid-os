"""
Unit tests for core control loop
"""

import pytest
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.control_loop import ControlLoop, ControlConfig, SystemState, SystemMetrics


class TestControlConfig:
    """Test ControlConfig"""
    
    def test_default_config(self):
        """Test default configuration"""
        config = ControlConfig()
        assert config.frequency_hz == 1000.0
        assert config.dt == 0.001
        assert config.enable_safety_checks
        assert config.enable_telemetry
    
    def test_custom_frequency(self):
        """Test custom frequency calculation"""
        config = ControlConfig(frequency_hz=500.0)
        assert config.dt == 0.002
        
        config = ControlConfig(frequency_hz=100.0)
        assert config.dt == 0.01


class TestSystemMetrics:
    """Test SystemMetrics"""
    
    def test_initialization(self):
        """Test metrics initialization"""
        metrics = SystemMetrics()
        assert metrics.cycle_time_ms == 0.0
        assert metrics.dropped_cycles == 0
        assert metrics.uptime_seconds == 0.0
    
    def test_update(self):
        """Test metrics update"""
        metrics = SystemMetrics()
        
        # Update with cycle time
        metrics.update(0.001)  # 1ms
        assert metrics.cycle_time_ms == 1.0
        assert metrics.control_frequency_hz == 1000.0
        
        # Update with longer cycle
        metrics.update(0.005)  # 5ms
        assert metrics.cycle_time_ms == 5.0
        assert metrics.max_cycle_time_ms == 5.0


class TestControlLoop:
    """Test ControlLoop"""
    
    def test_initialization(self):
        """Test control loop initialization"""
        loop = ControlLoop()
        assert loop.state == SystemState.INITIALIZING
        assert loop.config.frequency_hz == 1000.0
        assert not loop._running
    
    def test_subsystem_registration(self):
        """Test subsystem registration"""
        loop = ControlLoop()
        
        called = []
        
        def test_subsystem(dt, state):
            called.append(True)
            return {"test": "value"}
        
        loop.register_subsystem("test", test_subsystem)
        assert "test" in loop._subsystems
        
        # Execute step
        outputs = loop.step()
        assert len(called) == 1
        assert "test" in outputs
    
    def test_pre_post_callbacks(self):
        """Test pre and post step callbacks"""
        loop = ControlLoop()
        
        call_order = []
        
        def pre_callback(dt, state):
            call_order.append("pre")
        
        def subsystem(dt, state):
            call_order.append("subsystem")
            return {}
        
        def post_callback(dt, state, outputs):
            call_order.append("post")
        
        loop.add_pre_step_callback(pre_callback)
        loop.register_subsystem("test", subsystem)
        loop.add_post_step_callback(post_callback)
        
        loop.step()
        
        assert call_order == ["pre", "subsystem", "post"]
    
    def test_state_transition(self):
        """Test state transitions"""
        loop = ControlLoop()
        
        assert loop.state == SystemState.INITIALIZING
        
        loop.set_state(SystemState.IDLE)
        assert loop.state == SystemState.IDLE
        
        loop.set_state(SystemState.STANDING)
        assert loop.state == SystemState.STANDING
    
    def test_emergency_stop(self):
        """Test emergency stop"""
        loop = ControlLoop()
        
        loop.emergency_stop()
        assert loop._emergency_stop
        assert loop.state == SystemState.EMERGENCY_STOP
        
        # Step should return emergency stop flag
        outputs = loop.step()
        assert outputs.get("emergency_stop")
    
    def test_run_duration(self):
        """Test run with duration"""
        config = ControlConfig(frequency_hz=100.0)  # Lower freq for faster test
        loop = ControlLoop(config)
        
        step_count = []
        
        def counter(dt, state):
            step_count.append(1)
        
        loop.register_subsystem("counter", counter)
        
        # Run for 0.5 seconds
        start = time.time()
        loop.run(duration=0.5)
        elapsed = time.time() - start
        
        # Should have run approximately 50 steps (100 Hz * 0.5s)
        assert len(step_count) >= 40  # Allow some tolerance
        assert len(step_count) <= 60
        assert elapsed >= 0.5
        assert elapsed < 0.7  # Should not take much longer
    
    def test_error_handling(self):
        """Test subsystem error handling"""
        loop = ControlLoop()
        
        def failing_subsystem(dt, state):
            raise ValueError("Test error")
        
        loop.register_subsystem("failing", failing_subsystem)
        
        # Should handle error gracefully
        outputs = loop.step()
        
        # With safety checks, should trigger emergency stop
        if loop.config.enable_safety_checks:
            assert loop._emergency_stop
    
    def test_metrics_tracking(self):
        """Test that metrics are tracked"""
        config = ControlConfig(frequency_hz=1000.0)
        loop = ControlLoop(config)
        
        def slow_subsystem(dt, state):
            time.sleep(0.001)  # 1ms delay
            return {}
        
        loop.register_subsystem("slow", slow_subsystem)
        
        # Execute several steps
        for _ in range(10):
            loop.step()
        
        # Metrics should be updated
        assert loop.metrics.cycle_time_ms > 0
        assert loop.metrics.average_cycle_time_ms > 0
    
    def test_cycle_time_violation(self):
        """Test detection of cycle time violations"""
        config = ControlConfig(frequency_hz=1000.0, max_cycle_time_ms=2.0)
        loop = ControlLoop(config)
        
        def very_slow_subsystem(dt, state):
            time.sleep(0.003)  # 3ms - exceeds 2ms limit
            return {}
        
        loop.register_subsystem("slow", very_slow_subsystem)
        
        initial_dropped = loop.metrics.dropped_cycles
        loop.step()
        
        # Should have detected violation
        assert loop.metrics.dropped_cycles > initial_dropped


class TestSystemState:
    """Test SystemState enum"""
    
    def test_state_values(self):
        """Test that all states have correct values"""
        assert SystemState.INITIALIZING.value == "initializing"
        assert SystemState.IDLE.value == "idle"
        assert SystemState.STANDING.value == "standing"
        assert SystemState.WALKING.value == "walking"
        assert SystemState.BALANCING.value == "balancing"
        assert SystemState.FALLING.value == "falling"
        assert SystemState.EMERGENCY_STOP.value == "emergency_stop"
        assert SystemState.SHUTDOWN.value == "shutdown"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
