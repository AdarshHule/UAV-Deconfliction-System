"""
Distributed Architecture for UAV Deconfliction System
Designed to scale to 10,000+ drones with microservices, message queues, and distributed computing

This demonstrates the architectural patterns needed for production deployment
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
import hashlib
import json
import logging
from enum import Enum
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================================================================
# PART 1: MESSAGE QUEUE SYSTEM (Apache Kafka / RabbitMQ simulation)
# ============================================================================

class MessagePriority(Enum):
    """Message priority levels"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class Message:
    """Message for inter-service communication"""
    id: str
    topic: str
    payload: Dict
    priority: MessagePriority
    timestamp: str
    sender: str
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))


class MessageQueue(ABC):
    """Abstract message queue interface"""
    
    @abstractmethod
    def publish(self, topic: str, message: Message):
        pass
    
    @abstractmethod
    def subscribe(self, topic: str, callback):
        pass
    
    @abstractmethod
    def consume(self, topic: str) -> Optional[Message]:
        pass


class InMemoryMessageQueue(MessageQueue):
    """In-memory message queue for demonstration"""
    
    def __init__(self):
        self.queues: Dict[str, List[Message]] = {}
        self.subscribers: Dict[str, List] = {}
        logger.info("Initialized InMemoryMessageQueue")
    
    def publish(self, topic: str, message: Message):
        """Publish message to topic"""
        if topic not in self.queues:
            self.queues[topic] = []
        
        self.queues[topic].append(message)
        logger.debug(f"Published message to {topic}: {message.id}")
        
        # Notify subscribers
        if topic in self.subscribers:
            for callback in self.subscribers[topic]:
                callback(message)
    
    def subscribe(self, topic: str, callback):
        """Subscribe to topic"""
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)
        logger.info(f"Subscribed to topic: {topic}")
    
    def consume(self, topic: str) -> Optional[Message]:
        """Consume message from topic"""
        if topic in self.queues and self.queues[topic]:
            return self.queues[topic].pop(0)
        return None


# ============================================================================
# PART 2: SPATIAL PARTITIONING (Geohashing for distributed spatial queries)
# ============================================================================

class SpatialPartitioner:
    """Partition airspace into cells for distributed processing"""
    
    def __init__(self, cell_size: float = 1000.0):
        """
        Initialize spatial partitioner
        
        Args:
            cell_size: Size of each spatial cell in meters
        """
        self.cell_size = cell_size
        logger.info(f"Initialized SpatialPartitioner with cell_size={cell_size}m")
    
    def get_cell_id(self, x: float, y: float, z: float) -> str:
        """Get cell ID for a 3D position"""
        cell_x = int(x // self.cell_size)
        cell_y = int(y // self.cell_size)
        cell_z = int(z // self.cell_size)
        return f"cell_{cell_x}_{cell_y}_{cell_z}"
    
    def get_neighboring_cells(self, cell_id: str) -> List[str]:
        """Get neighboring cells (26 neighbors in 3D)"""
        parts = cell_id.split('_')
        cx, cy, cz = int(parts[1]), int(parts[2]), int(parts[3])
        
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neighbors.append(f"cell_{cx+dx}_{cy+dy}_{cz+dz}")
        
        return neighbors
    
    def assign_mission_to_cells(self, mission_waypoints) -> List[str]:
        """Assign a mission to spatial cells"""
        cells = set()
        for wp in mission_waypoints:
            # Handle both Waypoint objects and dictionaries
            if isinstance(wp, dict):
                x, y, z = wp.get('x', 0), wp.get('y', 0), wp.get('z', 0)
            else:
                x, y, z = wp.x, wp.y, wp.z
            
            cell_id = self.get_cell_id(x, y, z)
            cells.add(cell_id)
            # Add neighboring cells for safety buffer
            cells.update(self.get_neighboring_cells(cell_id))
        
        return list(cells)


# ============================================================================
# PART 3: DISTRIBUTED CACHE (Redis simulation)
# ============================================================================

class DistributedCache:
    """Distributed cache for flight data (Redis-like)"""
    
    def __init__(self, ttl_seconds: int = 300):
        """
        Initialize distributed cache
        
        Args:
            ttl_seconds: Time-to-live for cached entries
        """
        self.cache: Dict[str, Dict] = {}
        self.ttl_seconds = ttl_seconds
        logger.info(f"Initialized DistributedCache with TTL={ttl_seconds}s")
    
    def set(self, key: str, value: Dict, ttl: Optional[int] = None):
        """Set cache entry"""
        expiry = time.time() + (ttl or self.ttl_seconds)
        self.cache[key] = {
            'value': value,
            'expiry': expiry
        }
        logger.debug(f"Cached: {key}")
    
    def get(self, key: str) -> Optional[Dict]:
        """Get cache entry"""
        if key not in self.cache:
            return None
        
        entry = self.cache[key]
        if time.time() > entry['expiry']:
            del self.cache[key]
            return None
        
        return entry['value']
    
    def delete(self, key: str):
        """Delete cache entry"""
        if key in self.cache:
            del self.cache[key]
    
    def get_keys_by_pattern(self, pattern: str) -> List[str]:
        """Get keys matching pattern"""
        return [k for k in self.cache.keys() if pattern in k]


# ============================================================================
# PART 4: MICROSERVICES ARCHITECTURE
# ============================================================================

class MicroserviceBase(ABC):
    """Base class for all microservices"""
    
    def __init__(self, service_id: str, message_queue: MessageQueue):
        self.service_id = service_id
        self.message_queue = message_queue
        self.is_running = False
        logger.info(f"Initialized {self.__class__.__name__}: {service_id}")
    
    @abstractmethod
    def process_message(self, message: Message):
        """Process incoming message"""
        pass
    
    def start(self):
        """Start service"""
        self.is_running = True
        logger.info(f"Started service: {self.service_id}")
    
    def stop(self):
        """Stop service"""
        self.is_running = False
        logger.info(f"Stopped service: {self.service_id}")


class FlightDataIngestionService(MicroserviceBase):
    """Service for ingesting flight plan data"""
    
    def __init__(self, service_id: str, message_queue: MessageQueue, 
                 cache: DistributedCache, partitioner: SpatialPartitioner):
        super().__init__(service_id, message_queue)
        self.cache = cache
        self.partitioner = partitioner
        
        # Subscribe to topics
        self.message_queue.subscribe('flight.submit', self.process_message)
    
    def process_message(self, message: Message):
        """Process flight submission"""
        if message.topic == 'flight.submit':
            flight_data = message.payload
            flight_id = flight_data['id']
            
            logger.info(f"Ingesting flight: {flight_id}")
            
            # Store in cache
            self.cache.set(f"flight:{flight_id}", flight_data, ttl=3600)
            
            # Determine spatial cells
            cells = self.partitioner.assign_mission_to_cells(
                flight_data.get('waypoints', []))
            
            # Publish to spatial processing queues
            for cell_id in cells:
                routing_message = Message(
                    id=f"route_{flight_id}_{cell_id}",
                    topic=f"spatial.{cell_id}",
                    payload={'flight_id': flight_id, 'cell_id': cell_id},
                    priority=MessagePriority.NORMAL,
                    timestamp=datetime.now().isoformat(),
                    sender=self.service_id
                )
                self.message_queue.publish(f"spatial.{cell_id}", routing_message)
            
            logger.info(f"Flight {flight_id} assigned to {len(cells)} cells")


class ConflictDetectionService(MicroserviceBase):
    """Service for detecting conflicts in a spatial cell"""
    
    def __init__(self, service_id: str, cell_id: str, message_queue: MessageQueue,
                 cache: DistributedCache, safety_buffer: float = 50.0):
        super().__init__(service_id, message_queue)
        self.cell_id = cell_id
        self.cache = cache
        self.safety_buffer = safety_buffer
        
        # Subscribe to spatial topic
        self.message_queue.subscribe(f"spatial.{cell_id}", self.process_message)
    
    def process_message(self, message: Message):
        """Process conflict detection request"""
        if message.topic == f"spatial.{self.cell_id}":
            flight_id = message.payload['flight_id']
            
            logger.info(f"Checking conflicts for {flight_id} in {self.cell_id}")
            
            # Get flight data from cache
            flight_data = self.cache.get(f"flight:{flight_id}")
            if not flight_data:
                logger.warning(f"Flight data not found: {flight_id}")
                return
            
            # Get other flights in this cell
            other_flights = self._get_flights_in_cell()
            
            # Perform conflict detection (simplified)
            conflicts = self._detect_conflicts(flight_data, other_flights)
            
            if conflicts:
                # Publish conflict alert
                alert_message = Message(
                    id=f"conflict_{flight_id}",
                    topic='conflict.alert',
                    payload={
                        'flight_id': flight_id,
                        'conflicts': conflicts,
                        'cell_id': self.cell_id
                    },
                    priority=MessagePriority.HIGH,
                    timestamp=datetime.now().isoformat(),
                    sender=self.service_id
                )
                self.message_queue.publish('conflict.alert', alert_message)
                logger.warning(f"Conflicts detected for {flight_id}: {len(conflicts)}")
            else:
                logger.info(f"No conflicts for {flight_id} in {self.cell_id}")
    
    def _get_flights_in_cell(self) -> List[Dict]:
        """Get all flights assigned to this cell"""
        pattern = f"flight:"
        flight_keys = self.cache.get_keys_by_pattern(pattern)
        
        flights = []
        for key in flight_keys:
            flight_data = self.cache.get(key)
            if flight_data:
                flights.append(flight_data)
        
        return flights
    
    def _detect_conflicts(self, primary_flight: Dict, 
                         other_flights: List[Dict]) -> List[Dict]:
        """Simplified conflict detection"""
        conflicts = []
        
        # Simplified: just check if there are other flights
        # In production, this would be full spatio-temporal analysis
        if len(other_flights) > 1:  # More than just the primary
            conflicts.append({
                'type': 'potential_conflict',
                'cell_id': self.cell_id,
                'other_flights_count': len(other_flights) - 1
            })
        
        return conflicts


class ConflictResolutionService(MicroserviceBase):
    """Service for resolving detected conflicts"""
    
    def __init__(self, service_id: str, message_queue: MessageQueue,
                 cache: DistributedCache):
        super().__init__(service_id, message_queue)
        self.cache = cache
        
        # Subscribe to conflict alerts
        self.message_queue.subscribe('conflict.alert', self.process_message)
    
    def process_message(self, message: Message):
        """Process conflict alert and generate resolution"""
        if message.topic == 'conflict.alert':
            flight_id = message.payload['flight_id']
            conflicts = message.payload['conflicts']
            
            logger.info(f"Resolving conflicts for {flight_id}")
            
            # Generate alternative routes (simplified)
            resolution = self._generate_resolution(flight_id, conflicts)
            
            # Publish resolution
            resolution_message = Message(
                id=f"resolution_{flight_id}",
                topic='conflict.resolution',
                payload={
                    'flight_id': flight_id,
                    'resolution': resolution
                },
                priority=MessagePriority.HIGH,
                timestamp=datetime.now().isoformat(),
                sender=self.service_id
            )
            self.message_queue.publish('conflict.resolution', resolution_message)
            
            logger.info(f"Resolution generated for {flight_id}: {resolution['strategy']}")
    
    def _generate_resolution(self, flight_id: str, 
                            conflicts: List[Dict]) -> Dict:
        """Generate conflict resolution strategy"""
        # Simplified resolution strategies
        strategies = ['altitude_adjustment', 'time_delay', 'route_deviation']
        
        return {
            'flight_id': flight_id,
            'strategy': strategies[len(conflicts) % len(strategies)],
            'adjustment': 30,
            'confidence': 0.85
        }


class MonitoringService(MicroserviceBase):
    """Service for monitoring system health and metrics"""
    
    def __init__(self, service_id: str, message_queue: MessageQueue):
        super().__init__(service_id, message_queue)
        self.metrics = {
            'flights_processed': 0,
            'conflicts_detected': 0,
            'resolutions_generated': 0,
            'average_processing_time_ms': 0
        }
        
        # Subscribe to all topics for monitoring
        topics = ['flight.submit', 'conflict.alert', 'conflict.resolution']
        for topic in topics:
            self.message_queue.subscribe(topic, self.process_message)
    
    def process_message(self, message: Message):
        """Process message for monitoring"""
        if message.topic == 'flight.submit':
            self.metrics['flights_processed'] += 1
        elif message.topic == 'conflict.alert':
            self.metrics['conflicts_detected'] += 1
        elif message.topic == 'conflict.resolution':
            self.metrics['resolutions_generated'] += 1
        
        logger.debug(f"Metrics updated: {self.metrics}")
    
    def get_metrics(self) -> Dict:
        """Get current system metrics"""
        return self.metrics


# ============================================================================
# PART 5: LOAD BALANCER
# ============================================================================

class LoadBalancer:
    """Distribute workload across multiple service instances"""
    
    def __init__(self):
        self.service_instances: Dict[str, List[MicroserviceBase]] = {}
        self.round_robin_index: Dict[str, int] = {}
        logger.info("Initialized LoadBalancer")
    
    def register_service(self, service_type: str, service: MicroserviceBase):
        """Register a service instance"""
        if service_type not in self.service_instances:
            self.service_instances[service_type] = []
            self.round_robin_index[service_type] = 0
        
        self.service_instances[service_type].append(service)
        logger.info(f"Registered {service_type} instance: {service.service_id}")
    
    def get_service_instance(self, service_type: str) -> Optional[MicroserviceBase]:
        """Get next available service instance (round-robin)"""
        if service_type not in self.service_instances:
            return None
        
        instances = self.service_instances[service_type]
        if not instances:
            return None
        
        index = self.round_robin_index[service_type]
        service = instances[index]
        
        # Update round-robin index
        self.round_robin_index[service_type] = (index + 1) % len(instances)
        
        return service


# ============================================================================
# PART 6: ORCHESTRATION & DEMONSTRATION
# ============================================================================

class DistributedDeconflictionSystem:
    """Orchestrate all microservices"""
    
    def __init__(self, num_spatial_cells: int = 10):
        """Initialize distributed system"""
        logger.info("="*70)
        logger.info("INITIALIZING DISTRIBUTED UAV DECONFLICTION SYSTEM")
        logger.info("="*70)
        
        # Core infrastructure
        self.message_queue = InMemoryMessageQueue()
        self.cache = DistributedCache(ttl_seconds=3600)
        self.partitioner = SpatialPartitioner(cell_size=1000.0)
        self.load_balancer = LoadBalancer()
        
        # Initialize services
        self._initialize_services(num_spatial_cells)
        
        logger.info("System initialized successfully")
        logger.info("="*70)
    
    def _initialize_services(self, num_spatial_cells: int):
        """Initialize all microservices"""
        
        # Flight ingestion service
        ingestion_service = FlightDataIngestionService(
            'ingestion-001',
            self.message_queue,
            self.cache,
            self.partitioner
        )
        self.load_balancer.register_service('ingestion', ingestion_service)
        
        # Conflict detection services (one per spatial cell)
        for i in range(num_spatial_cells):
            cell_id = f"cell_{i}_0_0"  # Simplified cell IDs
            detection_service = ConflictDetectionService(
                f'detection-{i:03d}',
                cell_id,
                self.message_queue,
                self.cache,
                safety_buffer=50.0
            )
            self.load_balancer.register_service('detection', detection_service)
        
        # Conflict resolution service
        resolution_service = ConflictResolutionService(
            'resolution-001',
            self.message_queue,
            self.cache
        )
        self.load_balancer.register_service('resolution', resolution_service)
        
        # Monitoring service
        self.monitoring_service = MonitoringService(
            'monitoring-001',
            self.message_queue
        )
        
        logger.info(f"Initialized {num_spatial_cells + 3} microservices")
    
    def submit_flight_plan(self, flight_data: Dict):
        """Submit a flight plan for deconfliction"""
        message = Message(
            id=f"submit_{flight_data['id']}",
            topic='flight.submit',
            payload=flight_data,
            priority=MessagePriority.NORMAL,
            timestamp=datetime.now().isoformat(),
            sender='api-gateway'
        )
        
        self.message_queue.publish('flight.submit', message)
        logger.info(f"Submitted flight plan: {flight_data['id']}")
    
    def get_system_metrics(self) -> Dict:
        """Get system performance metrics"""
        return self.monitoring_service.get_metrics()
    
    def demonstrate_scalability(self):
        """Demonstrate system handling multiple flights"""
        logger.info("\n" + "="*70)
        logger.info("SCALABILITY DEMONSTRATION")
        logger.info("="*70)
        
        # Simulate 100 flight submissions
        num_flights = 100
        logger.info(f"Simulating {num_flights} concurrent flight submissions...")
        
        for i in range(num_flights):
            flight_data = {
                'id': f'FLIGHT-{i:04d}',
                'waypoints': [
                    {'x': i*10, 'y': i*10, 'z': 50, 'time': 0},
                    {'x': i*10 + 100, 'y': i*10 + 100, 'z': 50, 'time': 30}
                ],
                'time_window': (0, 30)
            }
            self.submit_flight_plan(flight_data)
        
        # Small delay for processing
        time.sleep(0.1)
        
        # Get metrics
        metrics = self.get_system_metrics()
        logger.info(f"\nSystem Metrics:")
        logger.info(f"  Flights Processed: {metrics['flights_processed']}")
        logger.info(f"  Conflicts Detected: {metrics['conflicts_detected']}")
        logger.info(f"  Resolutions Generated: {metrics['resolutions_generated']}")
        
        logger.info("\n✓ System successfully handled distributed workload")
        logger.info("="*70)


# ============================================================================
# PART 7: MAIN DEMONSTRATION
# ============================================================================

def main():
    """Demonstrate distributed architecture"""
    
    # Initialize system
    system = DistributedDeconflictionSystem(num_spatial_cells=10)
    
    # Submit sample flights
    logger.info("\n[1/2] Submitting sample flight plans...")
    
    flight1 = {
        'id': 'DEMO-001',
        'waypoints': [
            {'x': 0, 'y': 0, 'z': 50, 'time': 0},
            {'x': 100, 'y': 100, 'z': 50, 'time': 30}
        ],
        'time_window': (0, 30)
    }
    
    flight2 = {
        'id': 'DEMO-002',
        'waypoints': [
            {'x': 50, 'y': 50, 'z': 50, 'time': 0},
            {'x': 150, 'y': 150, 'z': 50, 'time': 30}
        ],
        'time_window': (0, 30)
    }
    
    system.submit_flight_plan(flight1)
    system.submit_flight_plan(flight2)
    
    # Demonstrate scalability
    logger.info("\n[2/2] Running scalability demonstration...")
    system.demonstrate_scalability()
    
    # Architecture summary
    logger.info("\n" + "="*70)
    logger.info("DISTRIBUTED ARCHITECTURE SUMMARY")
    logger.info("="*70)
    logger.info("✓ Message Queue: Apache Kafka / RabbitMQ pattern")
    logger.info("✓ Distributed Cache: Redis pattern")
    logger.info("✓ Spatial Partitioning: Geohashing for distributed queries")
    logger.info("✓ Microservices: Independent, scalable services")
    logger.info("✓ Load Balancer: Round-robin distribution")
    logger.info("✓ Monitoring: Real-time metrics collection")
    logger.info("\nSCALING TO 10,000+ DRONES:")
    logger.info("  • Horizontal scaling of detection services (add more instances)")
    logger.info("  • Database sharding by spatial cells")
    logger.info("  • CDN for static data distribution")
    logger.info("  • Auto-scaling based on load metrics")
    logger.info("  • Multi-region deployment for fault tolerance")
    logger.info("="*70)


if __name__ == "__main__":
    main()