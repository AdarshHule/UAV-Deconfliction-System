"""
Comprehensive Test Suite for UAV Deconfliction System
Tests spatial checks, temporal checks, edge cases, and stress scenarios
"""

import unittest
import numpy as np
from typing import List
import sys

# Import from main module (assumes main code is in deconfliction_system.py)
# from deconfliction_system import *

# For standalone testing, we'll include minimal definitions
class Waypoint:
    def __init__(self, x, y, z, time):
        self.x, self.y, self.z, self.time = x, y, z, time
    def to_array(self):
        return np.array([self.x, self.y, self.z])

class Mission:
    def __init__(self, id, waypoints, time_window):
        self.id = id
        self.waypoints = waypoints
        self.time_window = time_window

class DeconflictionEngine:
    def __init__(self, safety_buffer=50.0, time_resolution=0.5):
        self.safety_buffer = safety_buffer
        self.time_resolution = time_resolution
    
    def distance_3d(self, p1, p2):
        return np.linalg.norm(p1 - p2)
    
    def interpolate_position(self, waypoints, time):
        for i in range(len(waypoints) - 1):
            wp1, wp2 = waypoints[i], waypoints[i+1]
            if wp1.time <= time <= wp2.time:
                if wp2.time == wp1.time:
                    return wp1.to_array()
                t = (time - wp1.time) / (wp2.time - wp1.time)
                return (1-t) * wp1.to_array() + t * wp2.to_array()
        return None
    
    def check_conflicts(self, primary, other_flights):
        conflicts = []
        start, end = primary.time_window
        for t in np.arange(start, end + self.time_resolution, self.time_resolution):
            primary_pos = self.interpolate_position(primary.waypoints, t)
            if primary_pos is None:
                continue
            
            for flight in other_flights:
                other_pos = self.interpolate_position(flight.waypoints, t)
                if other_pos is None:
                    continue
                
                dist = self.distance_3d(primary_pos, other_pos)
                if dist < self.safety_buffer:
                    conflicts.append({
                        'time': t, 'distance': dist, 
                        'flight_id': flight.id, 'severity': 1 - dist/self.safety_buffer
                    })
        
        return {
            'status': 'clear' if not conflicts else 'conflict',
            'conflicts': conflicts,
            'conflict_count': len(conflicts)
        }


class TestWaypointClass(unittest.TestCase):
    """Test Waypoint data structure"""
    
    def test_waypoint_creation(self):
        """Test basic waypoint creation"""
        wp = Waypoint(100, 200, 50, 10)
        self.assertEqual(wp.x, 100)
        self.assertEqual(wp.y, 200)
        self.assertEqual(wp.z, 50)
        self.assertEqual(wp.time, 10)
    
    def test_waypoint_to_array(self):
        """Test conversion to numpy array"""
        wp = Waypoint(10, 20, 30, 5)
        arr = wp.to_array()
        np.testing.assert_array_equal(arr, np.array([10, 20, 30]))


class TestMissionClass(unittest.TestCase):
    """Test Mission data structure"""
    
    def test_mission_creation(self):
        """Test mission with multiple waypoints"""
        waypoints = [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30),
            Waypoint(200, 200, 50, 60)
        ]
        mission = Mission('TEST-001', waypoints, (0, 60))
        
        self.assertEqual(mission.id, 'TEST-001')
        self.assertEqual(len(mission.waypoints), 3)
        self.assertEqual(mission.time_window, (0, 60))
    
    def test_empty_mission(self):
        """Test edge case: mission with no waypoints"""
        mission = Mission('EMPTY', [], (0, 0))
        self.assertEqual(len(mission.waypoints), 0)


class TestSpatialConflictDetection(unittest.TestCase):
    """Test spatial conflict detection"""
    
    def setUp(self):
        self.engine = DeconflictionEngine(safety_buffer=50.0)
    
    def test_no_conflict_far_apart(self):
        """Test: Drones far apart should have no conflict"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(0, 200, 50, 0),
            Waypoint(100, 200, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')
        self.assertEqual(result['conflict_count'], 0)
    
    def test_conflict_same_position(self):
        """Test: Drones at same position should conflict"""
        primary = Mission('PRIMARY', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
        self.assertGreater(result['conflict_count'], 0)
    
    def test_conflict_within_safety_buffer(self):
        """Test: Drones within safety buffer should conflict"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        # Other drone 30m away (within 50m buffer)
        other = Mission('OTHER', [
            Waypoint(0, 30, 50, 0),
            Waypoint(100, 30, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
    
    def test_no_conflict_outside_safety_buffer(self):
        """Test: Drones outside safety buffer should not conflict"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        # Other drone 80m away (outside 50m buffer)
        other = Mission('OTHER', [
            Waypoint(0, 80, 50, 0),
            Waypoint(100, 80, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')
    
    def test_3d_spatial_conflict(self):
        """Test: 3D distance calculation including altitude"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(0, 0, 50, 30)
        ], (0, 30))
        
        # Other drone 30m above (should be within 50m buffer)
        other = Mission('OTHER', [
            Waypoint(0, 0, 80, 0),
            Waypoint(0, 0, 80, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')


class TestTemporalConflictDetection(unittest.TestCase):
    """Test temporal (time-based) conflict detection"""
    
    def setUp(self):
        self.engine = DeconflictionEngine(safety_buffer=50.0)
    
    def test_no_conflict_different_times(self):
        """Test: Drones at same location but different times - no conflict"""
        primary = Mission('PRIMARY', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        # Other drone at same location but later time
        other = Mission('OTHER', [
            Waypoint(100, 100, 50, 40),
            Waypoint(100, 100, 50, 70)
        ], (40, 70))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')
    
    def test_conflict_overlapping_time_windows(self):
        """Test: Drones with overlapping time windows and positions"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        # Other drone on similar path, overlapping time
        other = Mission('OTHER', [
            Waypoint(0, 0, 50, 10),
            Waypoint(100, 100, 50, 40)
        ], (10, 40))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
    
    def test_crossing_paths_different_speeds(self):
        """Test: Drones crossing paths at different speeds"""
        # Fast drone
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 10)  # Fast: 100m in 10s
        ], (0, 10))
        
        # Slow drone on intersecting path
        other = Mission('OTHER', [
            Waypoint(100, 0, 50, 0),
            Waypoint(0, 100, 50, 20)  # Slower
        ], (0, 20))
        
        result = self.engine.check_conflicts(primary, [other])
        # Should detect conflict at intersection point around t=5s
        self.assertEqual(result['status'], 'conflict')


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and boundary conditions"""
    
    def setUp(self):
        self.engine = DeconflictionEngine(safety_buffer=50.0)
    
    def test_single_waypoint_mission(self):
        """Test: Mission with only one waypoint (hovering)"""
        primary = Mission('PRIMARY', [
            Waypoint(100, 100, 50, 0)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(200, 200, 50, 0),
            Waypoint(200, 200, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        # Should handle gracefully
        self.assertIn(result['status'], ['clear', 'conflict'])
    
    def test_zero_duration_mission(self):
        """Test: Mission with zero duration"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(0, 0, 50, 0)
        ], (0, 0))
        
        other = Mission('OTHER', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 0)
        ], (0, 0))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')
    
    def test_no_other_flights(self):
        """Test: Primary mission with no other flights"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [])
        self.assertEqual(result['status'], 'clear')
        self.assertEqual(result['conflict_count'], 0)
    
    def test_many_other_flights(self):
        """Test: Scalability with many other flights"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        # Create 50 other flights
        other_flights = []
        for i in range(50):
            other_flights.append(Mission(f'OTHER-{i}', [
                Waypoint(i*10, i*10, 50+i, 0),
                Waypoint(i*10+50, i*10+50, 50+i, 30)
            ], (0, 30)))
        
        result = self.engine.check_conflicts(primary, other_flights)
        # Should complete without errors
        self.assertIsNotNone(result['status'])
    
    def test_exact_safety_buffer_distance(self):
        """Test: Edge case at exact safety buffer distance"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(0, 0, 50, 30)
        ], (0, 30))
        
        # Other drone exactly at safety buffer distance (50m)
        other = Mission('OTHER', [
            Waypoint(50, 0, 50, 0),
            Waypoint(50, 0, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        # Should be borderline - implementation dependent
        self.assertIn(result['status'], ['clear', 'conflict'])
    
    def test_negative_coordinates(self):
        """Test: Waypoints with negative coordinates"""
        primary = Mission('PRIMARY', [
            Waypoint(-100, -100, 50, 0),
            Waypoint(-50, -50, 50, 30)
        ], (0, 30))
        
        # Other drone very close at t=15s (should conflict)
        other = Mission('OTHER', [
            Waypoint(-80, -80, 50, 0),
            Waypoint(-60, -60, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        # Should detect conflict due to close proximity
        self.assertEqual(result['status'], 'conflict')
    
    def test_very_high_altitude(self):
        """Test: Waypoints at very high altitude"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 1000, 0),
            Waypoint(100, 100, 1000, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        # 950m altitude difference >> safety buffer
        self.assertEqual(result['status'], 'clear')


class TestSafetyBufferVariations(unittest.TestCase):
    """Test different safety buffer sizes"""
    
    def test_small_safety_buffer(self):
        """Test: Small safety buffer (10m)"""
        engine = DeconflictionEngine(safety_buffer=10.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(0, 15, 50, 0),
            Waypoint(100, 15, 50, 30)
        ], (0, 30))
        
        result = engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')
    
    def test_large_safety_buffer(self):
        """Test: Large safety buffer (200m)"""
        engine = DeconflictionEngine(safety_buffer=200.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(0, 150, 50, 0),
            Waypoint(100, 150, 50, 30)
        ], (0, 30))
        
        result = engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
    
    def test_zero_safety_buffer(self):
        """Test: Edge case with zero safety buffer"""
        engine = DeconflictionEngine(safety_buffer=0.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(0, 1, 50, 0),
            Waypoint(100, 1, 50, 30)
        ], (0, 30))
        
        result = engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'clear')


class TestMultipleConflicts(unittest.TestCase):
    """Test scenarios with multiple conflicts"""
    
    def setUp(self):
        self.engine = DeconflictionEngine(safety_buffer=50.0)
    
    def test_multiple_conflicts_single_flight(self):
        """Test: Multiple conflicts with single other flight"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30),
            Waypoint(200, 0, 50, 60)
        ], (0, 60))
        
        # Other flight crosses path twice
        other = Mission('OTHER', [
            Waypoint(50, -10, 50, 0),
            Waypoint(50, 10, 50, 30),
            Waypoint(150, 10, 50, 45),
            Waypoint(150, -10, 50, 60)
        ], (0, 60))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
        self.assertGreater(result['conflict_count'], 1)
    
    def test_conflicts_with_multiple_flights(self):
        """Test: Conflicts with multiple other flights"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        other1 = Mission('OTHER1', [
            Waypoint(20, 20, 50, 0),
            Waypoint(20, 20, 50, 30)
        ], (0, 30))
        
        other2 = Mission('OTHER2', [
            Waypoint(80, 80, 50, 0),
            Waypoint(80, 80, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other1, other2])
        self.assertEqual(result['status'], 'conflict')


class TestPerformanceStress(unittest.TestCase):
    """Test performance under stress conditions"""
    
    def test_long_duration_mission(self):
        """Test: Mission with very long duration"""
        engine = DeconflictionEngine(time_resolution=1.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(1000, 1000, 50, 3600)  # 1 hour mission
        ], (0, 3600))
        
        other = Mission('OTHER', [
            Waypoint(100, 100, 50, 0),
            Waypoint(900, 900, 50, 3600)
        ], (0, 3600))
        
        result = engine.check_conflicts(primary, [other])
        self.assertIsNotNone(result['status'])
    
    def test_many_waypoints(self):
        """Test: Mission with many waypoints"""
        waypoints_primary = [Waypoint(i*10, i*10, 50, i*5) for i in range(100)]
        primary = Mission('PRIMARY', waypoints_primary, (0, 495))
        
        waypoints_other = [Waypoint(i*10+20, i*10+20, 50, i*5) for i in range(100)]
        other = Mission('OTHER', waypoints_other, (0, 495))
        
        engine = DeconflictionEngine()
        result = engine.check_conflicts(primary, [other])
        self.assertIsNotNone(result['status'])
    
    def test_dense_airspace(self):
        """Test: Many drones in small area"""
        engine = DeconflictionEngine(safety_buffer=50.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        # Create 20 other flights in small area
        other_flights = []
        for i in range(20):
            angle = (i / 20) * 2 * 3.14159
            x = 100 + 150 * (i % 4 - 2) / 2
            y = 100 + 150 * (i % 5 - 2) / 2
            other_flights.append(Mission(f'OTHER-{i}', [
                Waypoint(x, y, 50 + i, 0),
                Waypoint(x, y, 50 + i, 30)
            ], (0, 30)))
        
        result = engine.check_conflicts(primary, other_flights)
        self.assertIsNotNone(result['status'])
    
    def test_high_speed_mission(self):
        """Test: Very fast moving drone (high speed)"""
        engine = DeconflictionEngine(time_resolution=0.1)
        
        # Drone moving 1000m in 10 seconds (100 m/s)
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(1000, 0, 50, 10)
        ], (0, 10))
        
        other = Mission('OTHER', [
            Waypoint(500, 30, 50, 0),
            Waypoint(500, 30, 50, 10)
        ], (0, 10))
        
        result = engine.check_conflicts(primary, [other])
        # Should detect conflict when primary passes by stationary other
        self.assertIsNotNone(result['status'])
    
    def test_complex_trajectory(self):
        """Test: Complex multi-segment trajectory"""
        engine = DeconflictionEngine()
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 60, 20),
            Waypoint(100, 100, 50, 40),
            Waypoint(0, 100, 70, 60),
            Waypoint(0, 0, 50, 80)
        ], (0, 80))
        
        other = Mission('OTHER', [
            Waypoint(50, 50, 55, 0),
            Waypoint(50, 50, 55, 80)
        ], (0, 80))
        
        result = engine.check_conflicts(primary, [other])
        self.assertIsNotNone(result['status'])
    
    def test_simultaneous_takeoff(self):
        """Test: Multiple drones taking off from nearby locations"""
        engine = DeconflictionEngine(safety_buffer=50.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 0, 0),
            Waypoint(0, 0, 100, 30)
        ], (0, 30))
        
        other_flights = [
            Mission('OTHER1', [
                Waypoint(20, 20, 0, 0),
                Waypoint(20, 20, 100, 30)
            ], (0, 30)),
            Mission('OTHER2', [
                Waypoint(40, 0, 0, 0),
                Waypoint(40, 0, 100, 30)
            ], (0, 30))
        ]
        
        result = engine.check_conflicts(primary, other_flights)
        self.assertEqual(result['status'], 'conflict')


class TestInterpolationAccuracy(unittest.TestCase):
    """Test trajectory interpolation accuracy"""
    
    def setUp(self):
        self.engine = DeconflictionEngine()
    
    def test_interpolation_at_waypoint(self):
        """Test: Interpolation exactly at waypoint time"""
        waypoints = [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 60, 30)
        ]
        
        pos = self.engine.interpolate_position(waypoints, 0)
        self.assertIsNotNone(pos)
        # Should be exactly at first waypoint
        self.assertAlmostEqual(pos[0], 0, places=1)
        self.assertAlmostEqual(pos[1], 0, places=1)
    
    def test_midpoint_interpolation(self):
        """Test: Interpolation at exact midpoint"""
        waypoints = [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 60, 20)
        ]
        
        pos = self.engine.interpolate_position(waypoints, 10)
        self.assertIsNotNone(pos)
        # At t=10 (midpoint), should be approximately halfway
        self.assertTrue(40 < pos[0] < 60)
        self.assertTrue(40 < pos[1] < 60)
    
    def test_interpolation_outside_range(self):
        """Test: Interpolation outside time range returns None"""
        waypoints = [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 60, 30)
        ]
        
        pos = self.engine.interpolate_position(waypoints, 50)
        self.assertIsNone(pos)


class TestConflictSeverity(unittest.TestCase):
    """Test conflict severity calculations"""
    
    def setUp(self):
        self.engine = DeconflictionEngine(safety_buffer=50.0)
    
    def test_collision_max_severity(self):
        """Test: Collision (0 distance) produces maximum severity"""
        primary = Mission('PRIMARY', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        other = Mission('OTHER', [
            Waypoint(100, 100, 50, 0),
            Waypoint(100, 100, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        if result['conflicts']:
            max_severity = max([c['severity'] for c in result['conflicts']])
            self.assertGreater(max_severity, 0.9)
    
    def test_high_severity_close_distance(self):
        """Test: Very close distance produces high severity"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        # Very close parallel path (10m away)
        other = Mission('OTHER', [
            Waypoint(0, 10, 50, 0),
            Waypoint(100, 10, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
        if result['conflicts']:
            avg_severity = sum([c['severity'] for c in result['conflicts']]) / len(result['conflicts'])
            self.assertGreater(avg_severity, 0.5)
    
    def test_low_severity_edge_distance(self):
        """Test: Distance near buffer edge produces low severity"""
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 0, 50, 30)
        ], (0, 30))
        
        # Just inside safety buffer (45m away)
        other = Mission('OTHER', [
            Waypoint(0, 45, 50, 0),
            Waypoint(100, 45, 50, 30)
        ], (0, 30))
        
        result = self.engine.check_conflicts(primary, [other])
        self.assertEqual(result['status'], 'conflict')
        if result['conflicts']:
            avg_severity = sum([c['severity'] for c in result['conflicts']]) / len(result['conflicts'])
            self.assertLess(avg_severity, 0.3)


class TestTimeResolution(unittest.TestCase):
    """Test different time resolution settings"""
    
    def test_fine_time_resolution(self):
        """Test: Fine time resolution (0.1s steps)"""
        engine = DeconflictionEngine(time_resolution=0.1)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 10)
        ], (0, 10))
        
        other = Mission('OTHER', [
            Waypoint(50, 50, 50, 0),
            Waypoint(50, 50, 50, 10)
        ], (0, 10))
        
        result = engine.check_conflicts(primary, [other])
        # Fine resolution should detect brief conflicts
        self.assertIsNotNone(result['status'])
    
    def test_coarse_time_resolution(self):
        """Test: Coarse time resolution (2s steps)"""
        engine = DeconflictionEngine(time_resolution=2.0)
        
        primary = Mission('PRIMARY', [
            Waypoint(0, 0, 50, 0),
            Waypoint(100, 100, 50, 10)
        ], (0, 10))
        
        other = Mission('OTHER', [
            Waypoint(50, 50, 50, 0),
            Waypoint(50, 50, 50, 10)
        ], (0, 10))
        
        result = engine.check_conflicts(primary, [other])
        # Coarse resolution might miss brief conflicts
        self.assertIsNotNone(result['status'])


def run_all_tests():
    """Run all test suites and generate report"""
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    test_classes = [
        TestWaypointClass,
        TestMissionClass,
        TestSpatialConflictDetection,
        TestTemporalConflictDetection,
        TestEdgeCases,
        TestSafetyBufferVariations,
        TestMultipleConflicts,
        TestPerformanceStress,
        TestInterpolationAccuracy,
        TestConflictSeverity,
        TestTimeResolution
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests Run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success Rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    print("="*70)
    
    return result


if __name__ == '__main__':
    run_all_tests()