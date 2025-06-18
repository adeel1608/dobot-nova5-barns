import psycopg2
import psycopg2.extras
from psycopg2.pool import SimpleConnectionPool
import os
from typing import List, Optional, Dict, Any
from . import models
import json
from datetime import datetime

# Connection pool
conn_pool = None

def connect():
    """Connect to PostgreSQL database and set up connection pool."""
    global conn_pool
    if conn_pool is not None:
        return
    
    # Database connection parameters from environment variables
    db_params = {
        'dbname': os.environ.get('POSTGRES_DB', os.environ.get('DB_NAME', 'barns_oms')),
        'user': os.environ.get('POSTGRES_USER', os.environ.get('DB_USER', 'barns_user')),
        'password': os.environ.get('POSTGRES_PASSWORD', os.environ.get('DB_PASSWORD', 'barns_pass')),
        'host': os.environ.get('POSTGRES_HOST', os.environ.get('DB_HOST', 'localhost')),
        'port': os.environ.get('POSTGRES_PORT', os.environ.get('DB_PORT', '5432'))
    }
    
    print(f"Connecting to database with params: {db_params}")
    
    # Create connection pool with min 1, max 10 connections
    conn_pool = SimpleConnectionPool(1, 10, **db_params)

def get_connection():
    """Get a connection from the pool."""
    if conn_pool is None:
        connect()
    return conn_pool.getconn()

def release_connection(conn):
    """Return a connection to the pool."""
    if conn_pool is not None:
        conn_pool.putconn(conn)

def save_order(order) -> int:
    """Save a new order to the database and return its ID.
    
    Args:
        order: The order object or dictionary to save
        
    Returns:
        int: The ID of the saved order
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            # Handle both object and dictionary formats
            if hasattr(order, 'status'):
                # Object format (models.Order)
                status = order.status
                cups = order.cups
            else:
                # Dictionary format (from RabbitMQ)
                status = order.get('status', 'queued')
                cups = order.get('cups', [])
            
            # Insert the order
            cur.execute(
                """
                INSERT INTO orders (status, created_at)
                VALUES (%s, %s)
                RETURNING id
                """,
                (status, datetime.now())
            )
            order_id = cur.fetchone()[0]
            
            # Insert order items (cups)
            for idx, cup in enumerate(cups):
                # Handle both object and dictionary formats for cups
                if hasattr(cup, 'addons'):
                    # Object format
                    addons_json = json.dumps(cup.addons) if cup.addons else '[]'
                    drink_type = cup.type
                    cup_size = cup.size
                else:
                    # Dictionary format
                    addons_json = json.dumps(cup.get('addons', cup.get('ingredients', [])))
                    drink_type = cup.get('type', 'unknown')
                    cup_size = cup.get('size', 'medium')
                
                cur.execute(
                    """
                    INSERT INTO order_items (order_id, cup_id, sequence_index, drink_type, cup_size, addons)
                    VALUES (%s, %s, %s, %s, %s, %s)
                    """,
                    (order_id, f"cup_{order_id}_{idx+1}", idx, drink_type, cup_size, addons_json)
                )
            
            conn.commit()
            return order_id
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def get_orders(status: Optional[str] = None) -> List[Dict[str, Any]]:
    """Retrieve orders from the database, optionally filtered by status.
    
    Args:
        status: Optional status filter
        
    Returns:
        List of order dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            if status:
                cur.execute(
                    """
                    SELECT id, created_at, status, started_at, completed_at, error_message
                    FROM orders
                    WHERE status = %s
                    ORDER BY created_at DESC
                    """,
                    (status,)
                )
            else:
                cur.execute(
                    """
                    SELECT id, created_at, status, started_at, completed_at, error_message
                    FROM orders
                    ORDER BY created_at DESC
                    """
                )
            orders = cur.fetchall()
            
            # Convert to regular dicts and handle datetime serialization
            result_orders = []
            for order in orders:
                order_dict = dict(order)
                # Convert datetime fields to strings
                for key, value in order_dict.items():
                    if isinstance(value, datetime):
                        order_dict[key] = value.isoformat()
                
                # Get order items
                cur.execute(
                    """
                    SELECT id, cup_id, sequence_index, drink_type, cup_size, addons
                    FROM order_items
                    WHERE order_id = %s
                    ORDER BY sequence_index
                    """,
                    (order_dict['id'],)
                )
                cups = cur.fetchall()
                order_dict['cups'] = [dict(cup) for cup in cups]
                result_orders.append(order_dict)
            
            return result_orders
    finally:
        release_connection(conn)

def get_order(order_id: int) -> Dict[str, Any]:
    """Retrieve a single order by its ID.
    
    Args:
        order_id: The ID of the order to retrieve
        
    Returns:
        Order dictionary
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            # Get order details
            cur.execute(
                """
                SELECT id, created_at, status, started_at, completed_at, error_message
                FROM orders
                WHERE id = %s
                """,
                (order_id,)
            )
            order = cur.fetchone()
            if not order:
                return None
            
            # Convert to regular dict and handle datetime serialization
            order = dict(order)
            for key, value in order.items():
                if isinstance(value, datetime):
                    order[key] = value.isoformat()
            
            # Get order items
            cur.execute(
                """
                SELECT id, cup_id, sequence_index, drink_type, cup_size, addons
                FROM order_items
                WHERE order_id = %s
                ORDER BY sequence_index
                """,
                (order_id,)
            )
            cups = cur.fetchall()
            order['cups'] = [dict(cup) for cup in cups]
            
            # Get tasks
            cur.execute(
                """
                SELECT id, arm_id, function_name, status, queued_at, started_at, completed_at, error_message
                FROM tasks
                WHERE order_id = %s
                """,
                (order_id,)
            )
            tasks = cur.fetchall()
            order['tasks'] = []
            for task in tasks:
                task_dict = dict(task)
                # Convert datetime fields to strings
                for key, value in task_dict.items():
                    if isinstance(value, datetime):
                        task_dict[key] = value.isoformat()
                order['tasks'].append(task_dict)
            
            return order
    finally:
        release_connection(conn)

def update_order_status(order_id: int, status: str, reason: Optional[str] = None) -> None:
    """Update the status of an existing order.
    
    Args:
        order_id: The ID of the order to update
        status: The new status to set
        reason: Optional reason/error message (stored in error_message field)
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            # Set appropriate timestamps based on status
            if status == 'processing':
                cur.execute(
                    """
                    UPDATE orders 
                    SET status = %s, started_at = %s, error_message = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), reason, order_id)
                )
            elif status in ('completed', 'error'):
                cur.execute(
                    """
                    UPDATE orders 
                    SET status = %s, completed_at = %s, error_message = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), reason, order_id)
                )
            else:
                # Just update status for other statuses
                cur.execute(
                    """
                    UPDATE orders 
                    SET status = %s, error_message = %s
                    WHERE id = %s
                    """,
                    (status, reason, order_id)
                )
            conn.commit()
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def delete_order(order_id: int) -> bool:
    """Delete an order and all its related data from the database.
    
    Args:
        order_id: The ID of the order to delete
        
    Returns:
        bool: True if the order was deleted successfully, False if order not found
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            # Check if order exists
            cur.execute("SELECT id FROM orders WHERE id = %s", (order_id,))
            if not cur.fetchone():
                return False
            
            # Delete related data in correct order (due to foreign key constraints)
            # Delete task steps first
            cur.execute(
                """
                DELETE FROM task_steps 
                WHERE task_id IN (SELECT id FROM tasks WHERE order_id = %s)
                """,
                (order_id,)
            )
            
            # Delete tasks
            cur.execute("DELETE FROM tasks WHERE order_id = %s", (order_id,))
            
            # Delete order items
            cur.execute("DELETE FROM order_items WHERE order_id = %s", (order_id,))
            
            # Delete the order itself
            cur.execute("DELETE FROM orders WHERE id = %s", (order_id,))
            
            conn.commit()
            return True
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def save_task(order_id: int, item_id: int, arm_id: int, function_name: str) -> int:
    """Save a new task to the database.
    
    Args:
        order_id: The ID of the order
        item_id: The ID of the order item (cup)
        arm_id: The ID of the arm (1 or 2)
        function_name: The name of the function to execute
        
    Returns:
        int: The ID of the saved task
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO tasks (order_id, item_id, arm_id, function_name, status, queued_at)
                VALUES (%s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                (order_id, item_id, arm_id, function_name, 'queued', datetime.now())
            )
            task_id = cur.fetchone()[0]
            conn.commit()
            return task_id
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def update_task_status(task_id: int, status: str, error_message: Optional[str] = None) -> None:
    """Update the status of a task.
    
    Args:
        task_id: The ID of the task
        status: The new status ('running', 'completed', 'failed')
        error_message: Optional error message if status is 'failed'
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            if status == 'running':
                cur.execute(
                    """
                    UPDATE tasks 
                    SET status = %s, started_at = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), task_id)
                )
            elif status in ('completed', 'failed'):
                cur.execute(
                    """
                    UPDATE tasks 
                    SET status = %s, completed_at = %s, error_message = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), error_message, task_id)
                )
            conn.commit()
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def save_task_step(task_id: int, step_index: int, step_type: str, function_name: str, 
                  params: Dict[str, Any], status: str = 'pending') -> int:
    """Save a task step to the database.
    
    Args:
        task_id: The ID of the parent task
        step_index: The index of the step in the sequence
        step_type: The type of step ('validation' or 'robot')
        function_name: The name of the function
        params: The parameters for the function
        status: The status of the step
        
    Returns:
        int: The ID of the saved step
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO task_steps (task_id, step_index, step_type, function_name, params, status)
                VALUES (%s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                (task_id, step_index, step_type, function_name, json.dumps(params), status)
            )
            step_id = cur.fetchone()[0]
            conn.commit()
            return step_id
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def update_task_step_status(step_id: int, status: str, error_message: Optional[str] = None) -> None:
    """Update the status of a task step.
    
    Args:
        step_id: The ID of the step
        status: The new status ('running', 'passed', 'failed')
        error_message: Optional error message if status is 'failed'
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            if status == 'running':
                cur.execute(
                    """
                    UPDATE task_steps 
                    SET status = %s, started_at = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), step_id)
                )
            elif status in ('passed', 'failed'):
                cur.execute(
                    """
                    UPDATE task_steps 
                    SET status = %s, completed_at = %s, error_message = %s
                    WHERE id = %s
                    """,
                    (status, datetime.now(), error_message, step_id)
                )
            conn.commit()
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def log_event(event_type: str, payload: Dict[str, Any]) -> int:
    """Log an event to the events table.
    
    Args:
        event_type: The type of event
        payload: The event payload
        
    Returns:
        int: The ID of the saved event
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO events (event_type, payload)
                VALUES (%s, %s)
                RETURNING id
                """,
                (event_type, json.dumps(payload))
            )
            event_id = cur.fetchone()[0]
            conn.commit()
            return event_id
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def create_alert(event_id: int, alert_type: str, severity: str) -> int:
    """Create an alert based on an event.
    
    Args:
        event_id: The ID of the event
        alert_type: The type of alert
        severity: The severity of the alert ('warning', 'critical')
        
    Returns:
        int: The ID of the created alert
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO alerts (event_id, alert_type, severity, acknowledged)
                VALUES (%s, %s, %s, %s)
                RETURNING id
                """,
                (event_id, alert_type, severity, False)
            )
            alert_id = cur.fetchone()[0]
            conn.commit()
            return alert_id
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def acknowledge_alert(alert_id: int) -> None:
    """Acknowledge an alert.
    
    Args:
        alert_id: The ID of the alert to acknowledge
    """
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                UPDATE alerts 
                SET acknowledged = %s, acknowledged_at = %s
                WHERE id = %s
                """,
                (True, datetime.now(), alert_id)
            )
            conn.commit()
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        release_connection(conn)

def get_active_alerts() -> List[Dict[str, Any]]:
    """Get all unacknowledged alerts.
    
    Returns:
        List of alert dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                """
                SELECT a.id, a.alert_type, a.severity, a.acknowledged, a.acknowledged_at,
                       e.event_type, e.payload, e.created_at
                FROM alerts a
                JOIN events e ON a.event_id = e.id
                WHERE a.acknowledged = %s
                ORDER BY e.created_at DESC
                """,
                (False,)
            )
            alerts = cur.fetchall()
            
            # Convert datetime objects to ISO format strings
            result_alerts = []
            for alert in alerts:
                alert_dict = dict(alert)
                for key, value in alert_dict.items():
                    if isinstance(value, datetime):
                        alert_dict[key] = value.isoformat()
                result_alerts.append(alert_dict)
            
            return result_alerts
    finally:
        release_connection(conn)

def get_acknowledged_alerts() -> List[Dict[str, Any]]:
    """Get all acknowledged alerts.
    
    Returns:
        List of acknowledged alert dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                """
                SELECT a.id, a.alert_type, a.severity, a.acknowledged, a.acknowledged_at,
                       e.event_type, e.payload, e.created_at
                FROM alerts a
                JOIN events e ON a.event_id = e.id
                WHERE a.acknowledged = %s
                ORDER BY a.acknowledged_at DESC
                """,
                (True,)
            )
            alerts = cur.fetchall()
            
            # Convert datetime objects to ISO format strings
            result_alerts = []
            for alert in alerts:
                alert_dict = dict(alert)
                for key, value in alert_dict.items():
                    if isinstance(value, datetime):
                        alert_dict[key] = value.isoformat()
                result_alerts.append(alert_dict)
            
            return result_alerts
    finally:
        release_connection(conn)

def get_order_tasks(order_id: int) -> List[Dict[str, Any]]:
    """Get all tasks for a specific order.
    
    Args:
        order_id: The ID of the order
        
    Returns:
        List of task dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                """
                SELECT id, order_id, item_id, arm_id, function_name, status, 
                       queued_at, started_at, completed_at, error_message
                FROM tasks
                WHERE order_id = %s
                ORDER BY queued_at
                """,
                (order_id,)
            )
            return cur.fetchall()
    finally:
        release_connection(conn)

def get_task_steps(task_id: int) -> List[Dict[str, Any]]:
    """Get all steps for a specific task.
    
    Args:
        task_id: The ID of the task
        
    Returns:
        List of task step dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                """
                SELECT id, task_id, step_index, step_type, function_name, params, status,
                       started_at, completed_at, error_message
                FROM task_steps
                WHERE task_id = %s
                ORDER BY step_index
                """,
                (task_id,)
            )
            return cur.fetchall()
    finally:
        release_connection(conn)

def get_recent_events(limit: int = 50) -> List[Dict[str, Any]]:
    """Get recent events from the database.
    
    Args:
        limit: Maximum number of events to return
        
    Returns:
        List of event dictionaries
    """
    conn = get_connection()
    try:
        with conn.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cur:
            cur.execute(
                """
                SELECT id, event_type, payload, created_at
                FROM events
                ORDER BY created_at DESC
                LIMIT %s
                """,
                (limit,)
            )
            return cur.fetchall()
    finally:
        release_connection(conn)
