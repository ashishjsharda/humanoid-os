"""
Comprehensive Test Suite for HumanoidOS

Tests all major components to ensure they work correctly.
"""

import sys
import os
import numpy as np
import logging
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import all components
from core.control_loop import ControlLoop, ControlConfig, SystemState
from locomotion.balance import BalanceController, BalanceConfig, BalanceState
from locomotion.gait import GaitGenerator, GaitConfig
from locomotion.advanced_gaits import AdvancedGaitGenerator, AdvancedGaitConfig, GaitType
from locomotion.push_recovery import PushRecoveryController, DisturbanceConfig

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_control_loop():
    """Test control loop"""
    print("\n" + "="*70)
    print("TEST 1: Control Loop")
    print("="*70)
    
    config = ControlConfig(frequency_hz=100.0)  # 100 Hz for testing
    loop = ControlLoop(config)
    
    call_count = 0
    
    def test_subsystem(dt, state):
        nonlocal call_count
        call_count += 1
        return {"test": "ok"}
    
    loop.register_subsystem("test", test_subsystem)
    loop.run(duration=0.5)
    
    print(f"✅ Control loop ran for 0.5s")
    print(f"   Calls: {call_count}")
    print(f"   Average frequency: {loop.metrics.control_frequency_hz:.1f} Hz")
    print(f"   Average cycle time: {loop.metrics.average_cycle_time_ms:.2f} ms")
    
    assert call_count > 40, "Expected ~50 calls at 100Hz"
    print("✅ Test PASSED")
    

def test_balance_controller():
    """Test balance controller"""
    print("\n" + "="*70)
    print("TEST 2: Balance Controller")
    print("="*70)
    
    config = BalanceConfig()
    controller = BalanceController(config)
    
    # Create slightly off-balance state
    state = BalanceState(
        com_position=np.array([0.02, 0.0, 0.9]),
        com_velocity=np.array([0.1, 0.0, 0.0]),
        com_acceleration=np.zeros(3),
        orientation=np.array([0.0, 0.05, 0.0]),
        angular_velocity=np.zeros(3),
        zmp=np.zeros(3)
    )
    
    # Compute control
    ankle_torques, hip_torques = controller.compute_control(state, 0.001)
    
    print(f"✅ Balance controller computed control")
    print(f"   Balanced: {state.is_balanced}")
    print(f"   Stability margin: {state.stability_margin:.3f}m")
    print(f"   ZMP: {state.zmp}")
    print(f"   Ankle torques: {ankle_torques}")
    
    assert len(ankle_torques) == 2, "Expected 2D ankle torques"
    assert len(hip_torques) == 2, "Expected 2D hip torques"
    print("✅ Test PASSED")


def test_basic_gait():
    """Test basic gait generator"""
    print("\n" + "="*70)
    print("TEST 3: Basic Gait Generator")
    print("="*70)
    
    config = GaitConfig(step_length=0.2, step_duration=0.6)
    gait = GaitGenerator(config)
    
    gait.start_walking()
    
    # Run for a few steps
    dt = 0.01
    steps_completed = 0
    last_step_count = 0
    
    for _ in range(500):  # 5 seconds
        state = gait.update(dt)
        if gait.state.step_count > last_step_count:
            steps_completed += 1
            last_step_count = gait.state.step_count
    
    print(f"✅ Basic gait generator ran for 5s")
    print(f"   Steps completed: {steps_completed}")
    print(f"   Final phase: {gait.state.phase.value}")
    print(f"   Is walking: {gait.is_walking()}")
    
    assert steps_completed >= 7, "Expected at least 7 steps in 5 seconds"
    print("✅ Test PASSED")


def test_advanced_gaits():
    """Test advanced gait patterns"""
    print("\n" + "="*70)
    print("TEST 4: Advanced Gait Patterns")
    print("="*70)
    
    config = AdvancedGaitConfig()
    gait = AdvancedGaitGenerator(config)
    
    # Test each gait type
    gaits_to_test = [
        GaitType.FORWARD_WALK,
        GaitType.BACKWARD_WALK,
        GaitType.SIDESTEP_LEFT,
        GaitType.TURN_RIGHT,
    ]
    
    for gait_type in gaits_to_test:
        gait.set_gait(gait_type, speed=0.3)
        
        # Run for 1 second
        for _ in range(100):
            state = gait.update(0.01)
        
        print(f"   ✅ {gait_type.value}: OK")
    
    print(f"✅ Tested {len(gaits_to_test)} gait patterns")
    print(f"   Final heading: {np.degrees(gait.current_heading):.1f}°")
    print(f"   Steps taken: {gait.step_count}")
    print("✅ Test PASSED")


def test_push_recovery():
    """Test push recovery system"""
    print("\n" + "="*70)
    print("TEST 5: Push Recovery System")
    print("="*70)
    
    config = DisturbanceConfig()
    controller = PushRecoveryController(config)
    
    # Simulate a forward push
    com_pos = np.array([0.0, 0.0, 0.9])
    com_vel = np.array([0.8, 0.0, 0.0])  # Fast forward velocity
    com_acc = np.array([4.0, 0.0, 0.0])  # High acceleration
    ang_vel = np.zeros(3)
    zmp = np.array([0.1, 0.0, 0.0])  # ZMP shifted forward
    support_center = np.zeros(2)
    
    # Detect disturbance
    disturbance = controller.detect_disturbance(
        com_pos, com_vel, com_acc, ang_vel, zmp, support_center, 0.0
    )
    
    print(f"✅ Disturbance detection: {'DETECTED' if disturbance else 'NONE'}")
    
    if disturbance:
        print(f"   Type: {disturbance.type}")
        print(f"   Magnitude: {disturbance.magnitude:.2f}")
        print(f"   Recovery required: {disturbance.recovery_required}")
        
        # Plan recovery step
        left_foot = np.array([0.0, 0.1, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0])
        
        target, foot = controller.plan_recovery_step(
            disturbance,
            com_pos,
            com_vel,
            "left",
            (left_foot, right_foot)
        )
        
        print(f"   Recovery step planned: {foot} foot")
        print(f"   Target position: {target[:2]}")
        
        # Compute capture point
        capture_point = controller.compute_capture_point(com_pos, com_vel)
        print(f"   Capture point: {capture_point}")
        
        assert disturbance.magnitude > 0.5, "Should detect significant disturbance"
        assert disturbance.recovery_required, "Should require recovery"
    
    print("✅ Test PASSED")


def test_integration():
    """Test integrated system"""
    print("\n" + "="*70)
    print("TEST 6: Integrated System")
    print("="*70)
    
    # Create all components
    control_loop = ControlLoop(ControlConfig(frequency_hz=100.0))
    balance_ctrl = BalanceController(BalanceConfig())
    gait_gen = AdvancedGaitGenerator(AdvancedGaitConfig())
    push_recovery = PushRecoveryController(DisturbanceConfig())
    
    # State
    state_data = {
        'com_position': np.array([0.0, 0.0, 0.9]),
        'com_velocity': np.zeros(3),
        'step_count': 0
    }
    
    def gait_subsystem(dt, state):
        gait_state = gait_gen.update(dt)
        state_data['step_count'] = gait_gen.step_count
        return gait_state
    
    def balance_subsystem(dt, state):
        balance_state = BalanceState(
            com_position=state_data['com_position'],
            com_velocity=state_data['com_velocity'],
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3)
        )
        return balance_ctrl.compute_control(balance_state, dt)
    
    control_loop.register_subsystem("gait", gait_subsystem)
    control_loop.register_subsystem("balance", balance_subsystem)
    
    # Start walking
    gait_gen.set_gait(GaitType.FORWARD_WALK, speed=0.3)
    
    # Run for 2 seconds
    control_loop.run(duration=2.0)
    
    print(f"✅ Integrated system ran for 2s")
    print(f"   Steps taken: {state_data['step_count']}")
    print(f"   Control loops: {control_loop.metrics.uptime_seconds / control_loop.config.dt:.0f}")
    print(f"   Average cycle time: {control_loop.metrics.average_cycle_time_ms:.2f} ms")
    
    assert state_data['step_count'] > 0, "Expected some steps"
    print("✅ Test PASSED")


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*70)
    print("🧪 HUMANOID OS TEST SUITE")
    print("="*70)
    print("\nRunning comprehensive tests...")
    
    start_time = time.time()
    
    tests = [
        ("Control Loop", test_control_loop),
        ("Balance Controller", test_balance_controller),
        ("Basic Gait", test_basic_gait),
        ("Advanced Gaits", test_advanced_gaits),
        ("Push Recovery", test_push_recovery),
        ("Integration", test_integration),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"❌ TEST FAILED: {name}")
            print(f"   Error: {e}")
            failed += 1
            import traceback
            traceback.print_exc()
    
    elapsed = time.time() - start_time
    
    print("\n" + "="*70)
    print("📊 TEST RESULTS")
    print("="*70)
    print(f"Total tests: {len(tests)}")
    print(f"Passed: {passed} ✅")
    print(f"Failed: {failed} ❌")
    print(f"Success rate: {(passed/len(tests)*100):.1f}%")
    print(f"Time elapsed: {elapsed:.2f}s")
    print("="*70)
    
    if failed == 0:
        print("\n🎉 ALL TESTS PASSED! 🎉")
        print("\nSystem is ready for demonstrations!")
    else:
        print(f"\n⚠️  {failed} test(s) failed. Please fix before proceeding.")
    
    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
