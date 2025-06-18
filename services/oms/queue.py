import os
import redis
from typing import Optional, List

# Redis connection
redis_client = None

def connect():
    """Connect to Redis."""
    global redis_client
    if redis_client is not None:
        return
    
    host = os.environ.get('REDIS_HOST', 'localhost')
    port = int(os.environ.get('REDIS_PORT', 6379))
    redis_client = redis.Redis(host=host, port=port, decode_responses=True)

def add_order(order_id: int) -> bool:
    """Add an order to the queue."""
    if redis_client is None:
        connect()
    
    # Add to the queue list
    redis_client.rpush('order_queue', str(order_id))
    
    # Also add to position mapping for faster lookups
    position = redis_client.llen('order_queue') - 1
    redis_client.hset('order_positions', str(order_id), position)
    
    return True

def remove(order_id: int) -> bool:
    """Remove an order from the queue."""
    if redis_client is None:
        connect()
    
    # Get current position
    current_pos = redis_client.hget('order_positions', str(order_id))
    if current_pos is None:
        return False
    
    # Remove from the queue list
    redis_client.lset('order_queue', int(current_pos), 'REMOVED')
    redis_client.lrem('order_queue', 1, 'REMOVED')
    
    # Remove from position mapping
    redis_client.hdel('order_positions', str(order_id))
    
    # Update positions for all remaining orders
    _update_positions()
    
    return True

def get_queue() -> List[int]:
    """Get the current order queue."""
    if redis_client is None:
        connect()
    
    # Get all order IDs as integers
    order_ids = redis_client.lrange('order_queue', 0, -1)
    return [int(order_id) for order_id in order_ids]

def get_position(order_id: int) -> Optional[int]:
    """Get the position of an order in the queue."""
    if redis_client is None:
        connect()
    
    position = redis_client.hget('order_positions', str(order_id))
    return int(position) if position is not None else None

def reorder(order_id: int, new_position: int) -> bool:
    """Change the position of an order in the queue."""
    if redis_client is None:
        connect()
    
    queue_length = redis_client.llen('order_queue')
    
    # Validate new position
    if new_position < 0 or new_position >= queue_length:
        return False
    
    # Get current position
    current_pos = redis_client.hget('order_positions', str(order_id))
    if current_pos is None:
        return False
    
    current_pos = int(current_pos)
    
    # If position is the same, no change needed
    if current_pos == new_position:
        return True
    
    # Get the order ID
    order_id_str = redis_client.lindex('order_queue', current_pos)
    
    # Remove from current position
    redis_client.lset('order_queue', current_pos, 'MOVED')
    redis_client.lrem('order_queue', 1, 'MOVED')
    
    # If new position is now at the end due to the removal
    if new_position > redis_client.llen('order_queue'):
        redis_client.rpush('order_queue', order_id_str)
    else:
        # Insert at the new position
        redis_client.linsert('order_queue', 'BEFORE', 
                           redis_client.lindex('order_queue', new_position), 
                           order_id_str)
    
    # Update positions for all orders
    _update_positions()
    
    return True

def bulk_reorder(order_ids: List[int]) -> bool:
    """Reorder the entire queue to match the provided order ID sequence."""
    if redis_client is None:
        connect()
    
    # Validate that all order IDs exist in the current queue
    current_queue = get_queue()
    order_ids_set = set(order_ids)
    current_queue_set = set(current_queue)
    
    # If the current queue is empty, just add all the order IDs
    if not current_queue:
        print(f"Queue is empty, adding {len(order_ids)} orders: {order_ids}")
        for i, order_id in enumerate(order_ids):
            redis_client.rpush('order_queue', str(order_id))
            redis_client.hset('order_positions', str(order_id), i)
        return True
    
    # If there's a mismatch, let's be more forgiving and sync the queue
    if order_ids_set != current_queue_set:
        missing_from_new = current_queue_set - order_ids_set
        extra_in_new = order_ids_set - current_queue_set
        
        print(f"Queue sync needed - Current: {current_queue}, New: {order_ids}")
        print(f"Missing from new: {missing_from_new}, Extra in new: {extra_in_new}")
        
        # Clear and rebuild the queue with the new order
        redis_client.delete('order_queue')
        redis_client.delete('order_positions')
        
        # Add all orders from the new sequence
        for i, order_id in enumerate(order_ids):
            redis_client.rpush('order_queue', str(order_id))
            redis_client.hset('order_positions', str(order_id), i)
        
        print(f"Queue rebuilt with {len(order_ids)} orders")
        return True
    
    # If sets match, just reorder normally
    redis_client.delete('order_queue')
    redis_client.delete('order_positions')
    
    # Add orders in the new sequence
    for i, order_id in enumerate(order_ids):
        redis_client.rpush('order_queue', str(order_id))
        redis_client.hset('order_positions', str(order_id), i)
    
    return True

def _update_positions():
    """Update the position mapping after queue changes."""
    if redis_client is None:
        connect()
    
    # Clear existing positions
    redis_client.delete('order_positions')
    
    # Update with new positions
    order_ids = redis_client.lrange('order_queue', 0, -1)
    for i, order_id in enumerate(order_ids):
        redis_client.hset('order_positions', order_id, i)

def sync_with_database():
    """Sync the Redis queue with queued orders in the database."""
    if redis_client is None:
        connect()
    
    try:
        # Import here to avoid circular imports
        from . import db
        
        # Get all queued orders from database
        queued_orders = db.get_orders(status='queued')
        queued_order_ids = [order['id'] for order in queued_orders]
        
        # Get current queue
        current_queue = get_queue()
        
        print(f"Syncing queue - DB has {len(queued_order_ids)} queued orders: {queued_order_ids}")
        print(f"Redis has {len(current_queue)} orders: {current_queue}")
        
        # If they don't match, rebuild from database
        if set(queued_order_ids) != set(current_queue):
            print("Queue out of sync with database, rebuilding...")
            
            # Clear current queue
            redis_client.delete('order_queue')
            redis_client.delete('order_positions')
            
            # Add all queued orders from database
            for i, order_id in enumerate(queued_order_ids):
                redis_client.rpush('order_queue', str(order_id))
                redis_client.hset('order_positions', str(order_id), i)
            
            print(f"Queue rebuilt with {len(queued_order_ids)} orders from database")
            return True
        else:
            print("Queue is in sync with database")
            return True
            
    except Exception as e:
        print(f"Error syncing queue with database: {e}")
        return False
