-- Initialize database schema for BARNS OMS

-- Connect to OMS database and set user context
\c barns_oms barns_user;

-- 1. ORDERS TABLE
CREATE TABLE IF NOT EXISTS orders (
  id              BIGSERIAL PRIMARY KEY,
  created_at      TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  status          VARCHAR(20) NOT NULL,          -- e.g. 'queued','processing','completed','error'
  started_at      TIMESTAMPTZ,
  completed_at    TIMESTAMPTZ,
  error_message   TEXT
);

-- 2. ORDER ITEMS (CUPS) TABLE
CREATE TABLE IF NOT EXISTS order_items (
  id              BIGSERIAL PRIMARY KEY,
  order_id        BIGINT NOT NULL REFERENCES orders(id) ON DELETE CASCADE,
  cup_id          TEXT NOT NULL,                 -- e.g. 'cup_001'
  sequence_index  INT NOT NULL,                  -- position in order (0-based)
  drink_type      TEXT NOT NULL,                 -- e.g. 'latte','espresso'
  cup_size        TEXT NOT NULL,                 -- e.g. 'small','medium','large'
  addons          JSONB DEFAULT '[]'::JSONB,     -- e.g. ["extra_shot","vanilla"]
  ingredients     JSONB DEFAULT '{}'::JSONB,     -- structured ingredients dict
  created_at      TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  -- Per-cup processing tracking: populated as the scheduler progresses
  cup_status      VARCHAR(20) DEFAULT 'pending', -- 'pending','processing','completed','failed','cancelled'
  cup_started_at  TIMESTAMPTZ,                   -- when the first scheduler task for this cup began
  cup_completed_at TIMESTAMPTZ,                  -- when the last scheduler task for this cup finished
  process_time_ms INTEGER,                       -- cup_completed_at - cup_started_at in milliseconds
  cup_error       TEXT                           -- error detail if cup_status = 'failed'
);

-- 3. TASKS TABLE
-- One high-level task per arm per cup (assigned by Scheduler)
CREATE TABLE IF NOT EXISTS tasks (
  id               BIGSERIAL PRIMARY KEY,
  order_id         BIGINT NOT NULL REFERENCES orders(id) ON DELETE CASCADE,
  item_id          BIGINT NOT NULL REFERENCES order_items(id) ON DELETE CASCADE,
  arm_id           INT NOT NULL,                 -- 1 or 2
  function_name    TEXT NOT NULL,                -- e.g. 'pull_espresso'
  status           VARCHAR(20) NOT NULL,         -- 'queued','running','completed','failed'
  queued_at        TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  started_at       TIMESTAMPTZ,
  completed_at     TIMESTAMPTZ,
  error_message    TEXT
);

-- 4. TASK STEPS LOG
-- Detailed logs of each micro-step within a task (from Routine service)
CREATE TABLE IF NOT EXISTS task_steps (
  id               BIGSERIAL PRIMARY KEY,
  task_id          BIGINT NOT NULL REFERENCES tasks(id) ON DELETE CASCADE,
  step_index       INT NOT NULL,                 -- order in sequence
  step_type        VARCHAR(10) NOT NULL,         -- 'validation' or 'robot'
  function_name    TEXT NOT NULL,                -- e.g. 'check_weight', 'activate_pump'
  params           JSONB NOT NULL,               -- actual parameters used
  status           VARCHAR(20) NOT NULL,         -- 'pending','running','passed','failed'
  started_at       TIMESTAMPTZ,
  completed_at     TIMESTAMPTZ,
  error_message    TEXT
);

-- 5. EVENTS TABLE
-- All published events for auditing/analytics
CREATE TABLE IF NOT EXISTS events (
  id               BIGSERIAL PRIMARY KEY,
  event_type       TEXT NOT NULL,                -- e.g. 'order.received','routine.completed'
  payload          JSONB NOT NULL,               -- full event JSON
  created_at       TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- 6. ALERTS TABLE
-- Critical alerts extracted from events (ingestion by Event Handler)
CREATE TABLE IF NOT EXISTS alerts (
  id               BIGSERIAL PRIMARY KEY,
  event_id         BIGINT NOT NULL REFERENCES events(id),
  alert_type       TEXT NOT NULL,                -- e.g. 'validation.failed','ingredient.low'
  severity         VARCHAR(10) NOT NULL,         -- e.g. 'warning','critical'
  acknowledged     BOOLEAN NOT NULL DEFAULT FALSE,
  acknowledged_at  TIMESTAMPTZ
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_orders_status ON orders(status);
CREATE INDEX IF NOT EXISTS idx_orders_created_at ON orders(created_at);
CREATE INDEX IF NOT EXISTS idx_order_items_order_id ON order_items(order_id);
CREATE INDEX IF NOT EXISTS idx_order_items_cup_status ON order_items(cup_status);
CREATE INDEX IF NOT EXISTS idx_tasks_order_id ON tasks(order_id);
CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status);
CREATE INDEX IF NOT EXISTS idx_task_steps_task_id ON task_steps(task_id);
CREATE INDEX IF NOT EXISTS idx_events_event_type ON events(event_type);
CREATE INDEX IF NOT EXISTS idx_events_created_at ON events(created_at);
CREATE INDEX IF NOT EXISTS idx_alerts_acknowledged ON alerts(acknowledged);

-- Insert some sample data for testing (optional)
-- INSERT INTO orders (status) VALUES ('queued'), ('processing'), ('completed'); 