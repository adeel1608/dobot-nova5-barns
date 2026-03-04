-- Migration 001: Add per-cup timing and status tracking to order_items
-- Run this against the barns_oms database for existing deployments.
-- Safe to run multiple times (all statements use IF NOT EXISTS / DO blocks).

\c barns_oms barns_user;

-- Add cup_status column (tracks the processing lifecycle of each cup)
ALTER TABLE order_items
  ADD COLUMN IF NOT EXISTS cup_status VARCHAR(20) DEFAULT 'pending';

-- Add cup_started_at (timestamp when the first scheduler task for this cup began)
ALTER TABLE order_items
  ADD COLUMN IF NOT EXISTS cup_started_at TIMESTAMPTZ;

-- Add cup_completed_at (timestamp when the last scheduler task for this cup finished)
ALTER TABLE order_items
  ADD COLUMN IF NOT EXISTS cup_completed_at TIMESTAMPTZ;

-- Add process_time_ms (computed duration in milliseconds: cup_completed_at - cup_started_at)
ALTER TABLE order_items
  ADD COLUMN IF NOT EXISTS process_time_ms INTEGER;

-- Add cup_error (error message when cup_status = 'failed')
ALTER TABLE order_items
  ADD COLUMN IF NOT EXISTS cup_error TEXT;

-- Index for filtering cups by status (e.g. find all failed cups across orders)
CREATE INDEX IF NOT EXISTS idx_order_items_cup_status ON order_items(cup_status);

-- Backfill cup_status for already-completed orders based on the parent order status.
-- Cups in completed orders are marked 'completed'; cups in failed/error orders are marked 'failed'.
-- Cups in other terminal states (stopped, cancelled) are marked 'cancelled'.
-- Non-terminal orders retain the default 'pending' status.
UPDATE order_items oi
SET cup_status = CASE
  WHEN o.status IN ('completed')          THEN 'completed'
  WHEN o.status IN ('error')              THEN 'failed'
  WHEN o.status IN ('stopped','cancelled') THEN 'cancelled'
  ELSE 'pending'
END
FROM orders o
WHERE oi.order_id = o.id
  AND oi.cup_status = 'pending';
