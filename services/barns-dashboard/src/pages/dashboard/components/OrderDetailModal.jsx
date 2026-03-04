import React, { useState, useEffect, useRef } from 'react';
import { useDashboardStore } from '../../../store/index';

// Stable fallback objects defined outside the component to avoid creating new references on every render.
const EMPTY_TASKS = { Arm1: [], Arm2: [] };
const EMPTY_TIMINGS = {};

// ---------------------------------------------------------------------------
// Helpers: mirror the logic already used in CoffeeProgressView
// ---------------------------------------------------------------------------

/**
 * Normalizes scheduler cup IDs (e.g. "34-1", "cup_1") to "cup-{N}".
 */
function normalizeCupId(rawCupId) {
  if (!rawCupId) return null;
  const lastSegment = String(rawCupId).split(/[-_]/).pop();
  const num = parseInt(lastSegment, 10);
  return isNaN(num) ? rawCupId : `cup-${num}`;
}

/**
 * Derives aggregate cup status from scheduler task list.
 * Priority: failed > in_progress > completed > pending
 */
function deriveCupStatus(cupId, schedulerTasks) {
  const allTasks = [
    ...(schedulerTasks.Arm1 || []),
    ...(schedulerTasks.Arm2 || []),
  ].filter(t => normalizeCupId(t.cup_id) === cupId);

  if (allTasks.length === 0) return 'pending';
  if (allTasks.some(t => t.status === 'failed')) return 'failed';
  if (allTasks.some(t => t.status === 'in_progress' || t.status === 'submitted')) return 'in_progress';
  if (allTasks.every(t => t.status === 'completed')) return 'completed';
  return 'pending';
}

/**
 * Returns elapsed milliseconds for a cup using taskTimings data.
 * Replicates the deriveCupElapsed logic from CoffeeProgressView.
 */
function deriveCupElapsedMs(cupId, schedulerTasks, taskTimings, currentTime) {
  const allTasks = [
    ...(schedulerTasks.Arm1 || []),
    ...(schedulerTasks.Arm2 || []),
  ].filter(t => normalizeCupId(t.cup_id) === cupId);

  if (allTasks.length === 0) return null;

  const timings = allTasks
    .map(task => taskTimings[`${task.cup_id}:${task.action}`])
    .filter(Boolean);

  if (timings.length === 0) return null;

  const hasTerminated = timings.some(tm => tm.elapsedTime != null);

  if (hasTerminated) {
    // Use the maximum frozen elapsed time among all tasks
    const elapsed = Math.max(...timings.map(tm => tm.elapsedTime || 0));
    return elapsed > 0 ? elapsed : null;
  }

  // Live calculation: span from earliest start to currentTime
  const starts = timings.map(tm => tm.startTime).filter(Boolean);
  if (starts.length === 0) return null;
  const earliestStart = Math.min(...starts);
  return currentTime - earliestStart;
}

/**
 * Formats milliseconds as "Xm Ys" or "Xs".
 */
function formatDuration(ms) {
  if (ms == null || ms < 0) return null;
  const totalSec = Math.floor(ms / 1000);
  const minutes = Math.floor(totalSec / 60);
  const seconds = totalSec % 60;
  if (minutes > 0) return `${minutes}m ${seconds}s`;
  return `${seconds}s`;
}

/**
 * Strips the "cup_" prefix from size strings (e.g. "cup_H12" -> "H12").
 */
function sanitizeSize(raw) {
  if (!raw) return null;
  return raw.replace(/^cup[_-]/i, '').trim() || null;
}

// ---------------------------------------------------------------------------
// Sub-components
// ---------------------------------------------------------------------------

function CupStatusDot({ status }) {
  const map = {
    completed:   'bg-green-500',
    failed:      'bg-red-500',
    in_progress: 'bg-blue-500 animate-pulse',
    pending:     'bg-gray-300',
  };
  return (
    <span
      className={`inline-block w-2.5 h-2.5 rounded-full flex-shrink-0 ${map[status] || map.pending}`}
      title={status}
    />
  );
}

function CupStatusLabel({ status }) {
  const variants = {
    completed:   'bg-green-100 text-green-800',
    failed:      'bg-red-100 text-red-800',
    in_progress: 'bg-blue-100 text-blue-800',
    pending:     'bg-gray-100 text-gray-500',
  };
  const labels = {
    completed:   'Done',
    failed:      'Failed',
    in_progress: 'Running',
    pending:     'Pending',
  };
  return (
    <span className={`text-xs font-semibold px-2 py-0.5 rounded-full ${variants[status] || variants.pending}`}>
      {labels[status] || 'Pending'}
    </span>
  );
}

/**
 * Renders an expandable cup card.
 */
function CupCard({ cup, index, cupStatus, elapsedMs, isExpanded, onToggle, t }) {
  const drinkName = cup.drink_type || cup.type || (t ? t('unknownItem') : 'Unknown');
  const size = sanitizeSize(cup.cup_size || cup.size);
  const addons = Array.isArray(cup.addons) ? cup.addons.filter(a => a && String(a).trim()) : [];
  const ingredients = cup.ingredients && typeof cup.ingredients === 'object'
    ? Object.entries(cup.ingredients)
    : [];
  // DB-persisted error detail (available for historical orders)
  const cupError = cup.cup_error || null;

  const headerBg = {
    completed:   'bg-green-50 hover:bg-green-100 border-l-4 border-l-green-400',
    failed:      'bg-red-50 hover:bg-red-100 border-l-4 border-l-red-400',
    in_progress: 'bg-blue-50 hover:bg-blue-100 border-l-4 border-l-blue-400',
    cancelled:   'bg-gray-50 hover:bg-gray-100 border-l-4 border-l-gray-300',
    pending:     'bg-amber-50 hover:bg-amber-100 border-l-4 border-l-amber-300',
  };

  return (
    <div className="border border-gray-200 rounded-lg overflow-hidden">
      {/* Cup header row -- always visible, click to toggle */}
      <button
        type="button"
        onClick={onToggle}
        className={`w-full flex items-center gap-2 px-3 py-2.5 transition-colors text-left ${headerBg[cupStatus] || headerBg.pending}`}
      >
        <CupStatusDot status={cupStatus} />

        <span className="font-semibold text-gray-800 text-sm">
          {t ? t('cup') : 'Cup'} #{index + 1}
        </span>

        <span className="text-gray-700 text-sm truncate flex-1">{drinkName}</span>

        {size && (
          <span className="text-xs font-medium bg-gray-200 text-gray-700 px-1.5 py-0.5 rounded font-mono flex-shrink-0">
            {size}
          </span>
        )}

        <CupStatusLabel status={cupStatus} />

        {elapsedMs != null && (
          <span className="text-xs text-gray-500 font-mono flex-shrink-0">
            {formatDuration(elapsedMs)}
          </span>
        )}

        {/* Chevron */}
        <svg
          className={`w-4 h-4 text-gray-400 flex-shrink-0 transition-transform ${isExpanded ? 'rotate-180' : ''}`}
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
        </svg>
      </button>

      {/* Expanded detail */}
      {isExpanded && (
        <div className="px-4 py-3 bg-white space-y-3 text-sm border-t border-gray-100">
          {/* Drink + size row */}
          <div className="grid grid-cols-2 gap-2">
            <div>
              <p className="text-xs text-gray-500 font-medium uppercase tracking-wide mb-0.5">
                {t ? t('drink') : 'Drink'}
              </p>
              <p className="text-gray-900 font-medium">{drinkName}</p>
            </div>
            {size && (
              <div>
                <p className="text-xs text-gray-500 font-medium uppercase tracking-wide mb-0.5">
                  {t ? t('size') : 'Size'}
                </p>
                <p className="text-gray-900 font-medium">{size}</p>
              </div>
            )}
          </div>

          {/* Addons */}
          {addons.length > 0 && (
            <div>
              <p className="text-xs text-gray-500 font-medium uppercase tracking-wide mb-1">
                {t ? t('addons') : 'Addons'}
              </p>
              <div className="flex flex-wrap gap-1">
                {addons.map((addon, i) => (
                  <span
                    key={i}
                    className="text-xs bg-indigo-50 text-indigo-700 border border-indigo-200 px-2 py-0.5 rounded-full"
                  >
                    {String(addon)}
                  </span>
                ))}
              </div>
            </div>
          )}

          {/* Ingredients */}
          {ingredients.length > 0 && (
            <div>
              <p className="text-xs text-gray-500 font-medium uppercase tracking-wide mb-1">
                Ingredients
              </p>
              <div className="grid grid-cols-2 gap-x-4 gap-y-1">
                {ingredients.map(([key, value]) => (
                  <div key={key} className="flex justify-between text-xs">
                    <span className="text-gray-600 capitalize">{key.replace(/_/g, ' ')}</span>
                    <span className="text-gray-900 font-mono font-medium">
                      {typeof value === 'object' ? JSON.stringify(value) : String(value)}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          )}

          {addons.length === 0 && ingredients.length === 0 && (
            <p className="text-xs text-gray-400 italic">No addons or ingredient overrides.</p>
          )}

          {/* DB-stored error detail for failed cups */}
          {cupError && (
            <div className="flex items-start gap-1.5 text-xs text-red-700 bg-red-50 border border-red-200 rounded p-2">
              <svg className="w-3.5 h-3.5 flex-shrink-0 mt-0.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              <span>{cupError}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

// ---------------------------------------------------------------------------
// Main component
// ---------------------------------------------------------------------------

/**
 * OrderDetailModal
 *
 * Compact (1024x768-friendly) modal showing full order details:
 * - Summary bar: status, total order time, cup count
 * - Scrollable section: order info, expandable cup cards, timeline
 * - Sticky footer: action buttons
 *
 * Props:
 *   order          - raw order object from the store (not pre-formatted)
 *   onClose        - () => void
 *   onStartOrder   - (id) => void
 *   onResumeOrder  - (id) => void
 *   getStatusBadge - (status) => ReactNode
 *   t              - translation function
 */
export default function OrderDetailModal({ order, onClose, onStartOrder, onResumeOrder, getStatusBadge, t }) {
  const [expandedCups, setExpandedCups] = useState(new Set());
  const [currentTime, setCurrentTime] = useState(Date.now());
  const overlayRef = useRef(null);

  // Use three separate selectors so each returns a stable primitive/reference.
  // A combined object selector would create a new object on every render, triggering an infinite loop.
  const schedulerCurrentOrderId = useDashboardStore(state => state.schedulerCurrentOrderId);
  const rawSchedulerTasks = useDashboardStore(state => state.schedulerTasks);
  const rawTaskTimings = useDashboardStore(state => state.taskTimings);
  const schedulerTasks = rawSchedulerTasks || EMPTY_TASKS;
  const taskTimings = rawTaskTimings || EMPTY_TIMINGS;

  // Live clock for in-progress cup timers
  useEffect(() => {
    const isLive = order && ['PROCESSING', 'STOPPING'].includes(order.status?.toUpperCase());
    if (!isLive) return;
    const id = setInterval(() => setCurrentTime(Date.now()), 1000);
    return () => clearInterval(id);
  }, [order]);

  // Close on Escape key
  useEffect(() => {
    const handler = (e) => { if (e.key === 'Escape') onClose(); };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [onClose]);

  if (!order) return null;

  const status = order.status?.toUpperCase() || 'QUEUED';
  const cups = order.cups || [];

  // Determine if this is the currently tracked order (has per-cup scheduler data)
  const isCurrentOrder = schedulerCurrentOrderId != null && order.id === schedulerCurrentOrderId;

  // ---------------------------------------------------------------------------
  // Total order duration
  // ---------------------------------------------------------------------------
  const startTs = order.started_at ? new Date(order.started_at).getTime() : null;
  const endTs = order.completed_at ? new Date(order.completed_at).getTime() : null;
  let totalDurationMs = null;
  if (startTs) {
    totalDurationMs = (endTs || currentTime) - startTs;
  }

  // ---------------------------------------------------------------------------
  // Per-cup data
  // Priority: live scheduler state (current order) > DB-persisted cup_status > order-level fallback
  // ---------------------------------------------------------------------------
  const cupData = cups.map((cup, idx) => {
    const cupId = `cup-${idx + 1}`;

    let cupStatus;
    let elapsedMs = null;

    if (isCurrentOrder) {
      // Live: derive from in-memory scheduler tasks
      cupStatus = deriveCupStatus(cupId, schedulerTasks);
      elapsedMs = deriveCupElapsedMs(cupId, schedulerTasks, taskTimings, currentTime);
    } else if (cup.cup_status && cup.cup_status !== 'pending') {
      // Historical: use DB-stored cup_status
      cupStatus = cup.cup_status;
      elapsedMs = cup.process_time_ms != null ? cup.process_time_ms : null;
    } else {
      // Fallback: derive from order-level status
      cupStatus = status === 'COMPLETED' ? 'completed'
        : status === 'ERROR' ? 'failed'
        : status === 'CANCELLED' || status === 'STOPPED' ? 'cancelled'
        : 'pending';
    }

    return { cup, cupId, cupStatus, elapsedMs };
  });

  // ---------------------------------------------------------------------------
  // Toggle cup expand
  // ---------------------------------------------------------------------------
  const toggleCup = (idx) => {
    setExpandedCups(prev => {
      const next = new Set(prev);
      if (next.has(idx)) { next.delete(idx); } else { next.add(idx); }
      return next;
    });
  };

  // ---------------------------------------------------------------------------
  // Timeline timestamps
  // ---------------------------------------------------------------------------
  const formatTs = (iso) => iso ? new Date(iso).toLocaleString() : null;
  const createdAt = formatTs(order.created_at);
  const startedAt = formatTs(order.started_at);
  const completedAt = formatTs(order.completed_at);
  const updatedAt = completedAt || startedAt || createdAt;

  // Cup outcome counts for the section header
  const completedCups = cupData.filter(d => d.cupStatus === 'completed').length;
  const failedCups = cupData.filter(d => d.cupStatus === 'failed').length;
  const cancelledCups = cupData.filter(d => d.cupStatus === 'cancelled').length;

  // ---------------------------------------------------------------------------
  // Render
  // ---------------------------------------------------------------------------
  return (
    <div
      ref={overlayRef}
      className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-3"
      onClick={(e) => { if (e.target === overlayRef.current) onClose(); }}
    >
      <div className="bg-white rounded-xl w-full max-w-2xl max-h-[88vh] flex flex-col shadow-2xl overflow-hidden">

        {/* ----------------------------------------------------------------- */}
        {/* Sticky Header                                                      */}
        {/* ----------------------------------------------------------------- */}
        <div className="flex items-center gap-3 px-4 py-3 border-b border-gray-200 bg-gradient-to-r from-blue-50 to-indigo-50 flex-shrink-0">
          <div className="p-1.5 bg-blue-500 rounded-md flex-shrink-0">
            <svg className="w-4 h-4 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
            </svg>
          </div>

          <div className="flex-1 min-w-0">
            <h2 className="text-base font-bold text-gray-900 leading-tight">
              {t ? t('orderId') : 'Order ID'} #{order.id}
            </h2>
            <p className="text-xs text-gray-500">
              {cups.length} cup{cups.length !== 1 ? 's' : ''}
              {order.created_at && (
                <span className="ml-1.5 text-gray-400">
                  &middot; {new Date(order.created_at).toLocaleDateString()}
                </span>
              )}
            </p>
          </div>

          <div className="flex items-center gap-2 flex-shrink-0">
            {getStatusBadge(status)}
            <button
              onClick={onClose}
              className="p-1.5 hover:bg-gray-100 rounded-md transition-colors"
              title="Close"
            >
              <svg className="w-4 h-4 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
        </div>

        {/* ----------------------------------------------------------------- */}
        {/* Summary bar                                                        */}
        {/* ----------------------------------------------------------------- */}
        <div className="flex items-center gap-4 px-4 py-2 bg-gray-50 border-b border-gray-100 text-xs flex-shrink-0 flex-wrap">
          {/* Cup count */}
          <div className="flex items-center gap-1 text-gray-600">
            <svg className="w-3.5 h-3.5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
            </svg>
            <span className="font-medium">{cups.length} cup{cups.length !== 1 ? 's' : ''}</span>
          </div>

          {/* Total duration */}
          {totalDurationMs != null && (
            <div className="flex items-center gap-1 text-gray-600">
              <svg className="w-3.5 h-3.5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              <span className="font-medium font-mono">{formatDuration(totalDurationMs)}</span>
              <span className="text-gray-400">total</span>
            </div>
          )}

          {/* Outcome pills — only shown when we have meaningful data */}
          {completedCups > 0 && (
            <span className="inline-flex items-center gap-1 text-green-700 bg-green-50 border border-green-200 px-1.5 py-0.5 rounded-full font-medium">
              <span className="w-1.5 h-1.5 rounded-full bg-green-500 inline-block" />
              {completedCups} done
            </span>
          )}
          {failedCups > 0 && (
            <span className="inline-flex items-center gap-1 text-red-700 bg-red-50 border border-red-200 px-1.5 py-0.5 rounded-full font-medium">
              <span className="w-1.5 h-1.5 rounded-full bg-red-500 inline-block" />
              {failedCups} failed
            </span>
          )}
          {cancelledCups > 0 && (
            <span className="inline-flex items-center gap-1 text-gray-500 bg-gray-100 border border-gray-200 px-1.5 py-0.5 rounded-full font-medium">
              <span className="w-1.5 h-1.5 rounded-full bg-gray-400 inline-block" />
              {cancelledCups} cancelled
            </span>
          )}

          {/* Manual required flag */}
          {cups.some(c => Array.isArray(c.addons) && c.addons.includes('manual_required')) && (
            <span className="inline-flex items-center gap-1 text-amber-700 bg-amber-50 border border-amber-200 px-1.5 py-0.5 rounded-full font-medium">
              <svg className="w-3 h-3" fill="currentColor" viewBox="0 0 20 20">
                <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
              </svg>
              Manual required
            </span>
          )}
        </div>

        {/* Error banner — shown below summary bar, only when there is an order-level error */}
        {order.error_message && (
          <div className="flex items-start gap-2 px-4 py-2 bg-red-50 border-b border-red-100 text-xs text-red-700 flex-shrink-0">
            <svg className="w-3.5 h-3.5 flex-shrink-0 mt-0.5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            <span className="break-words">{order.error_message}</span>
          </div>
        )}

        {/* ----------------------------------------------------------------- */}
        {/* Scrollable body                                                    */}
        {/* ----------------------------------------------------------------- */}
        <div className="flex-1 overflow-y-auto min-h-0">
          <div className="p-4 space-y-4">

            {/* Order Information */}
            <section>
              <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                <svg className="w-3.5 h-3.5 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
                {t ? t('orderInformation') : 'Order Information'}
              </h3>
              <div className="bg-gray-50 rounded-lg divide-y divide-gray-100 text-sm">
                <div className="flex justify-between items-center px-3 py-2">
                  <span className="text-gray-500">{t ? t('orderId') : 'Order ID'}</span>
                  <span className="font-mono font-semibold text-gray-900">#{order.id}</span>
                </div>
                <div className="flex justify-between items-center px-3 py-2">
                  <span className="text-gray-500">{t ? t('status') : 'Status'}</span>
                  {getStatusBadge(status)}
                </div>
                <div className="flex justify-between items-center px-3 py-2">
                  <span className="text-gray-500">Cups</span>
                  <span className="font-medium text-gray-900">{cups.length} cup{cups.length !== 1 ? 's' : ''}</span>
                </div>
              </div>
            </section>

            {/* Cups (expandable) */}
            {cups.length > 0 && (
              <section>
                <div className="flex items-center gap-1.5 mb-2">
                  <svg className="w-3.5 h-3.5 text-amber-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
                  </svg>
                  <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider">
                    {t ? t('orderDetails') : 'Order Details'}
                  </h3>
                  <span className="ml-auto text-xs text-gray-400 normal-case flex items-center gap-1">
                    <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                    </svg>
                    expand
                  </span>
                </div>
                <div className="space-y-2">
                  {cupData.map(({ cup, cupStatus, elapsedMs }, idx) => (
                    <CupCard
                      key={idx}
                      cup={cup}
                      index={idx}
                      cupStatus={cupStatus}
                      elapsedMs={elapsedMs}
                      isExpanded={expandedCups.has(idx)}
                      onToggle={() => toggleCup(idx)}
                      t={t}
                    />
                  ))}
                </div>
              </section>
            )}

            {/* Timeline */}
            <section>
              <h3 className="text-xs font-semibold text-gray-500 uppercase tracking-wider mb-2 flex items-center gap-1.5">
                <svg className="w-3.5 h-3.5 text-green-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
                {t ? t('timeline') : 'Timeline'}
              </h3>
              <div className="bg-gray-50 rounded-lg divide-y divide-gray-100 text-sm">
                {createdAt && (
                  <div className="flex justify-between items-center px-3 py-2">
                    <span className="text-gray-500">{t ? t('created') : 'Created'}</span>
                    <span className="font-mono text-gray-900 text-xs">{createdAt}</span>
                  </div>
                )}
                {startedAt && (
                  <div className="flex justify-between items-center px-3 py-2">
                    <span className="text-gray-500">{t ? t('started') : 'Started'}</span>
                    <span className="font-mono text-gray-900 text-xs">{startedAt}</span>
                  </div>
                )}
                {completedAt && (
                  <div className="flex justify-between items-center px-3 py-2">
                    <span className="text-gray-500">{t ? t('completed') : 'Completed'}</span>
                    <span className="font-mono text-gray-900 text-xs">{completedAt}</span>
                  </div>
                )}
                {updatedAt && updatedAt !== completedAt && (
                  <div className="flex justify-between items-center px-3 py-2">
                    <span className="text-gray-500">{t ? t('lastUpdated') : 'Last Updated'}</span>
                    <span className="font-mono text-gray-900 text-xs">{updatedAt}</span>
                  </div>
                )}
                {totalDurationMs != null && (
                  <div className="flex justify-between items-center px-3 py-2">
                    <span className="text-gray-500">Total Duration</span>
                    <span className="font-mono font-semibold text-gray-900 text-xs">
                      {formatDuration(totalDurationMs)}
                    </span>
                  </div>
                )}
              </div>
            </section>

          </div>
        </div>

        {/* ----------------------------------------------------------------- */}
        {/* Sticky Footer                                                      */}
        {/* ----------------------------------------------------------------- */}
        <div className="flex justify-end gap-2 px-4 py-3 border-t border-gray-200 bg-gray-50 flex-shrink-0">
          <button
            onClick={onClose}
            className="px-3 py-1.5 text-sm text-gray-700 bg-white border border-gray-300 rounded-lg font-medium hover:bg-gray-50 transition-colors"
          >
            {t ? t('close') : 'Close'}
          </button>

          {status === 'QUEUED' && (
            <button
              onClick={() => { onStartOrder(order.id); onClose(); }}
              className="px-3 py-1.5 text-sm bg-blue-600 hover:bg-blue-700 text-white rounded-lg font-medium transition-colors"
            >
              {t ? t('startOrder') : 'Start Order'}
            </button>
          )}

          {status === 'HALTED' && (
            <button
              onClick={() => { onResumeOrder(order.id); onClose(); }}
              className="px-3 py-1.5 text-sm bg-orange-600 hover:bg-orange-700 text-white rounded-lg font-medium transition-colors"
            >
              {t ? t('resumeOrder') : 'Resume Order'}
            </button>
          )}

          {status === 'ERROR' && (
            <button
              onClick={() => { onStartOrder(order.id); onClose(); }}
              className="px-3 py-1.5 text-sm bg-red-600 hover:bg-red-700 text-white rounded-lg font-medium transition-colors"
            >
              {t ? t('retryOrder') : 'Retry Order'}
            </button>
          )}
        </div>

      </div>
    </div>
  );
}
