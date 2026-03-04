import React, { useRef, useState, useEffect, useCallback } from 'react';

const ARROW_H = 26;

/**
 * Normalizes cup IDs from any format to a stable "cup-{N}" key.
 *
 * The scheduler emits cup IDs as "{orderId}-{cupNum}" (e.g. "34-1"),
 * while this component addresses cups by 1-based index ("cup-1", "cup-2").
 * Extracting the last numeric segment from either format yields the same index.
 */
function normalizeCupId(rawCupId) {
  if (!rawCupId) return null;
  const lastSegment = String(rawCupId).split(/[-_]/).pop();
  const num = parseInt(lastSegment, 10);
  return isNaN(num) ? rawCupId : `cup-${num}`;
}

/**
 * Derives an aggregate status for a single cup by inspecting all scheduler
 * tasks (from both arms) that reference the given cup_id (normalized).
 *
 * Priority: failed > in_progress > completed > pending
 */
function deriveCupStatus(cupId, schedulerTasks) {
  const allTasks = [
    ...(schedulerTasks.Arm1 || []),
    ...(schedulerTasks.Arm2 || []),
  ].filter(t => normalizeCupId(t.cup_id) === cupId);

  if (allTasks.length === 0) return 'pending';
  if (allTasks.some(t => t.status === 'failed')) return 'failed';
  if (allTasks.some(t => t.status === 'in_progress' || t.status === 'submitted'))
    return 'in_progress';
  if (allTasks.every(t => t.status === 'completed')) return 'completed';
  return 'pending';
}

/**
 * Sanitizes size strings that may be raw hardware IDs (e.g. "cup_H12" -> "H12").
 * Returns null if the value is empty or uninformative.
 */
function sanitizeSize(raw) {
  if (!raw) return null;
  const cleaned = raw.replace(/^cup[_-]/i, '').trim();
  return cleaned || null;
}

function CupStatusIcon({ status }) {
  const base = 'w-10 h-10 rounded-full flex items-center justify-center flex-shrink-0';

  if (status === 'completed') {
    return (
      <div className={`${base} bg-green-500`}>
        <svg className="w-5 h-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="3">
          <path strokeLinecap="round" strokeLinejoin="round" d="M5 13l4 4L19 7" />
        </svg>
      </div>
    );
  }
  if (status === 'failed') {
    return (
      <div className={`${base} bg-red-500`}>
        <svg className="w-5 h-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2.5">
          <path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
        </svg>
      </div>
    );
  }
  if (status === 'in_progress') {
    return (
      <div className={`${base} bg-amber-500`}>
        <svg className="animate-spin w-5 h-5 text-white" fill="none" viewBox="0 0 24 24">
          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
        </svg>
      </div>
    );
  }
  return (
    <div className={`${base} bg-gray-100`}>
      <svg className="w-5 h-5 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="1.5">
        <path strokeLinecap="round" strokeLinejoin="round" d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
      </svg>
    </div>
  );
}

function StatusBadge({ status, t, compact }) {
  const configs = {
    completed: { bg: 'bg-green-50', text: 'text-green-700', dot: 'bg-green-500', label: t('completedLabel') },
    in_progress: { bg: 'bg-amber-50', text: 'text-amber-700', dot: 'bg-amber-400', label: t('inProgress') },
    failed: { bg: 'bg-red-50', text: 'text-red-700', dot: 'bg-red-500', label: t('failed') },
    pending: { bg: 'bg-gray-50', text: 'text-gray-400', dot: 'bg-gray-300', label: t('pending') },
  };
  const c = configs[status] || configs.pending;
  if (compact) {
    return <span className={`w-3 h-3 rounded-full flex-shrink-0 ${c.dot}`} />;
  }
  return (
    <span className={`inline-flex items-center gap-1.5 text-[11px] font-semibold ${c.bg} ${c.text} rounded-full px-2.5 py-1 whitespace-nowrap`}>
      <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${c.dot}`} />
      {c.label}
    </span>
  );
}

/**
 * Formats elapsed milliseconds as "M:SS" (e.g. 65000 -> "1:05").
 * Returns null when no valid duration is provided.
 */
function formatElapsed(ms) {
  if (ms == null || ms < 0) return null;
  const totalSeconds = Math.floor(ms / 1000);
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}:${String(seconds).padStart(2, '0')}`;
}

function CupCard({ cup, idx, cupId, status, isArm1Active, isArm2Active, registerRef, t, compact, elapsedMs }) {
  const drinkName = cup.drink_type || cup.type || t('unknownItem');
  const displaySize = sanitizeSize(cup.cup_size || cup.size || '');
  const isActive = isArm1Active || isArm2Active;
  const elapsedLabel = formatElapsed(elapsedMs);

  const cardVariant = (() => {
    if (status === 'failed') return 'bg-red-50 border-red-200 shadow-red-50';
    if (status === 'completed') return 'bg-green-50 border-green-200';
    if (status === 'in_progress') return 'bg-amber-50 border-amber-200 shadow-amber-100';
    return 'bg-white border-gray-200';
  })();

  return (
    <div
      ref={el => registerRef(cupId, el)}
      className={`relative flex items-center gap-3 px-4 py-3.5 rounded-2xl border transition-all duration-200 overflow-hidden ${cardVariant}${isActive ? ' shadow-lg' : ' shadow-sm'}`}
    >
      {/* Arm 1 left accent strip */}
      {isArm1Active && (
        <div className="absolute left-0 inset-y-0 w-1 rounded-l-2xl bg-blue-500" />
      )}
      {/* Arm 2 right accent strip */}
      {isArm2Active && (
        <div className="absolute right-0 inset-y-0 w-1 rounded-r-2xl bg-purple-500" />
      )}

      <CupStatusIcon status={status} />

      <div className="flex-1 min-w-0">
        <div className="font-bold text-sm text-gray-900 truncate leading-snug">{drinkName}</div>
        {drinkName.toLowerCase() !== 'calibrate' && (
          <div className="flex items-center gap-1.5 mt-1.5 overflow-hidden">
            <span className="text-[10px] font-semibold text-gray-400 bg-gray-100 rounded-md px-2 py-0.5 whitespace-nowrap">
              {t('cup')} {idx + 1}
            </span>
            {displaySize && (
              <span className="text-[10px] font-semibold text-gray-400 bg-gray-100 rounded-md px-2 py-0.5 whitespace-nowrap">
                {displaySize}
              </span>
            )}
            {elapsedLabel && (status === 'in_progress' || status === 'completed') && (
              <span className={`text-[10px] font-semibold rounded-md px-2 py-0.5 whitespace-nowrap flex items-center gap-1 ${
                status === 'in_progress'
                  ? 'text-amber-700 bg-amber-100'
                  : 'text-green-700 bg-green-100'
              }`}>
                <svg className="w-2.5 h-2.5 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2.5">
                  <circle cx="12" cy="12" r="10" />
                  <path strokeLinecap="round" d="M12 6v6l4 2" />
                </svg>
                {elapsedLabel}
              </span>
            )}
          </div>
        )}
      </div>

      <div className="flex-shrink-0">
        <StatusBadge status={status} t={t} compact={compact} />
      </div>
    </div>
  );
}

/**
 * Renders a narrow vertical track with a colored dashed rail and a smoothly
 * sliding arrow indicator — no text label, just a visual cue.
 *
 * direction: 'right' = Arm 1 track on the left side, arrow points right
 *            'left'  = Arm 2 track on the right side, arrow points left
 */
function ArmTrack({ railRef, arrowY, visible, direction, color }) {
  const arrowPoints = direction === 'right' ? '2,3 18,12 2,21' : '18,3 2,12 18,21';

  return (
    <div className="flex-shrink-0 w-5 flex flex-col items-center py-1">
      <div ref={railRef} className="flex-1 relative w-full flex justify-center overflow-hidden">
        {/* Dashed rail line */}
        <div
          className="absolute top-0 bottom-0"
          style={{
            left: '50%',
            transform: 'translateX(-50%)',
            width: 1,
            borderLeft: `2px dashed ${color}40`,
          }}
        />

        {/* Sliding arrow indicator */}
        <div
          style={{
            position: 'absolute',
            top: 0,
            left: '50%',
            width: ARROW_H,
            height: ARROW_H,
            transform: `translateX(-50%) translateY(${arrowY}px)`,
            transition: 'transform 0.45s cubic-bezier(0.34, 1.4, 0.64, 1), opacity 0.25s ease',
            opacity: visible ? 1 : 0,
            pointerEvents: 'none',
            filter: visible ? `drop-shadow(0 0 5px ${color}99)` : 'none',
          }}
        >
          <svg width={ARROW_H} height={ARROW_H} viewBox="0 0 20 24" fill={color}>
            <polygon points={arrowPoints} />
          </svg>
        </div>
      </div>
    </div>
  );
}

/**
 * CoffeeProgressView
 *
 * Shows each cup in the active order as a card in a centered column.
 * Left track: Arm 1 colored arrow slides to the cup Arm 1 is processing.
 * Right track: Arm 2 colored arrow slides to the cup Arm 2 is processing.
 * Cup cards are vertically centered in the available space.
 */
/**
 * Derives the total elapsed time (ms) for a cup from taskTimings.
 *
 * - For in-progress cups: uses currentTime minus the earliest startTime.
 * - For completed/failed cups: uses the frozen elapsedTime already stored.
 * - Returns null when no timing data exists.
 */
function deriveCupElapsed(cupId, schedulerTasks, taskTimings, currentTime) {
  const allTasks = [
    ...(schedulerTasks.Arm1 || []),
    ...(schedulerTasks.Arm2 || []),
  ].filter(t => normalizeCupId(t.cup_id) === cupId);

  if (allTasks.length === 0) return null;

  const timings = allTasks
    .map(task => taskTimings[`${task.cup_id}:${task.action}`])
    .filter(Boolean);

  if (timings.length === 0) return null;

  const earliestStart = Math.min(...timings.map(t => t.startTime).filter(Boolean));
  if (!earliestStart) return null;

  // If any timing is still live (elapsedTime === null), use currentTime
  const hasLive = timings.some(t => t.elapsedTime === null);
  if (hasLive) return currentTime - earliestStart;

  // All frozen: latest end time minus earliest start
  const latestEnd = Math.max(
    ...timings.map(t => t.startTime + t.elapsedTime).filter(Boolean)
  );
  return latestEnd - earliestStart;
}

export default function CoffeeProgressView({ cups, schedulerTasks, t, compact, taskTimings, currentTime }) {
  const outerRef = useRef(null);
  const arm1RailRef = useRef(null);
  const arm2RailRef = useRef(null);
  const cupListRef = useRef(null);
  const cupRefs = useRef({});

  const [arm1ArrowY, setArm1ArrowY] = useState(0);
  const [arm2ArrowY, setArm2ArrowY] = useState(0);

  // Raw cup IDs from scheduler (e.g. "34-1"), normalized to "cup-1" for ref lookups
  const arm1RawCupId =
    (schedulerTasks.Arm1 || []).find(
      task => task.status === 'in_progress' || task.status === 'submitted'
    )?.cup_id ?? null;

  const arm2RawCupId =
    (schedulerTasks.Arm2 || []).find(
      task => task.status === 'in_progress' || task.status === 'submitted'
    )?.cup_id ?? null;

  const arm1ActiveCupId = normalizeCupId(arm1RawCupId);
  const arm2ActiveCupId = normalizeCupId(arm2RawCupId);

  const calculateArrowPositions = useCallback(() => {
    if (arm1ActiveCupId && arm1RailRef.current) {
      const el = cupRefs.current[arm1ActiveCupId];
      // Guard: skip detached elements — getBoundingClientRect returns zeros when unmounted
      if (el && el.isConnected) {
        const railRect = arm1RailRef.current.getBoundingClientRect();
        const elRect = el.getBoundingClientRect();
        setArm1ArrowY(elRect.top - railRect.top + el.offsetHeight / 2 - ARROW_H / 2);
      }
    }
    if (arm2ActiveCupId && arm2RailRef.current) {
      const el = cupRefs.current[arm2ActiveCupId];
      // Guard: skip detached elements — getBoundingClientRect returns zeros when unmounted
      if (el && el.isConnected) {
        const railRect = arm2RailRef.current.getBoundingClientRect();
        const elRect = el.getBoundingClientRect();
        setArm2ArrowY(elRect.top - railRect.top + el.offsetHeight / 2 - ARROW_H / 2);
      }
    }
  }, [arm1ActiveCupId, arm2ActiveCupId]);

  useEffect(() => {
    calculateArrowPositions();

    const outer = outerRef.current;
    const cupList = cupListRef.current;
    const resizeObserver = new ResizeObserver(calculateArrowPositions);
    if (outer) resizeObserver.observe(outer);
    if (cupList) cupList.addEventListener('scroll', calculateArrowPositions);

    return () => {
      resizeObserver.disconnect();
      if (cupList) cupList.removeEventListener('scroll', calculateArrowPositions);
    };
  }, [calculateArrowPositions]);

  const registerCupRef = useCallback((cupId, el) => {
    if (el) {
      cupRefs.current[cupId] = el;
    } else {
      delete cupRefs.current[cupId];
    }
  }, []);

  if (!cups || cups.length === 0) {
    return (
      <div className="flex-1 flex flex-col items-center justify-center gap-3 text-gray-200">
        <svg className="w-12 h-12" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="1">
          <path strokeLinecap="round" strokeLinejoin="round" d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
        </svg>
        <span className="text-sm font-medium text-gray-300">{t('noTasksYet')}</span>
      </div>
    );
  }

  return (
    <div ref={outerRef} className="flex flex-row flex-1 gap-2 overflow-hidden min-h-0 h-full">
      {/* Left track — Arm 1 (blue) */}
      <ArmTrack
        railRef={arm1RailRef}
        arrowY={arm1ArrowY}
        visible={!!arm1ActiveCupId}
        direction="right"
        color="#3B82F6"
      />

      {/* Cup list — scrollable, content vertically centered */}
      <div
        ref={cupListRef}
        className="flex-1 overflow-y-auto"
        style={{ scrollbarWidth: 'none' }}
      >
        {/* CSS grid with alignContent:center keeps cups centered when shorter than container */}
        <div
          style={{
            display: 'grid',
            alignContent: 'start',
            gap: 10,
            minHeight: '100%',
            paddingTop: 4,
            paddingBottom: 4,
          }}
        >
          {cups.map((cup, idx) => {
            const cupId = `cup-${idx + 1}`;
            const status = deriveCupStatus(cupId, schedulerTasks);
            const elapsedMs = taskTimings && currentTime
              ? deriveCupElapsed(cupId, schedulerTasks, taskTimings, currentTime)
              : null;

            return (
              <CupCard
                key={cupId}
                cup={cup}
                idx={idx}
                cupId={cupId}
                status={status}
                isArm1Active={arm1ActiveCupId === cupId}
                isArm2Active={arm2ActiveCupId === cupId}
                registerRef={registerCupRef}
                t={t}
                compact={compact}
                elapsedMs={elapsedMs}
              />
            );
          })}
        </div>
      </div>

      {/* Right track — Arm 2 (purple) */}
      <ArmTrack
        railRef={arm2RailRef}
        arrowY={arm2ArrowY}
        visible={!!arm2ActiveCupId}
        direction="left"
        color="#8B5CF6"
      />
    </div>
  );
}
