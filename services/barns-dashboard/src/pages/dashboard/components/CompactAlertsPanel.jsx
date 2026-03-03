/**
 * CompactAlertsPanel
 * POS-optimized alerts panel for the compact dashboard.
 * Denser layout, smaller text, touch-friendly acknowledge buttons.
 * Reuses all business logic from AlertsPanel but with a slimmer UI.
 */

import React, { useState, useEffect, useRef } from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';
import { SERVICE_OFFLINE_MESSAGE } from '../../../utils/errorHandler';
import { getValidationMessage, getValidationData } from '../../../constants/validationMessages.jsx';
import { getIngredientData, getSeverityMessage } from '../../../constants/ingredientMappings.jsx';

export default function CompactAlertsPanel() {
    const { t } = useTranslation('alerts');
    const {
        alerts,
        acknowledgeAlert,
        fetchAlerts,
        isLoading,
        errors,
        clearError,
        navigateToTab
    } = useStore();

    const [acknowledging, setAcknowledging] = useState(new Set());
    const audioContextRef = useRef(null);
    const oscillator1Ref = useRef(null);
    const gainRef = useRef(null);
    const isBuzzingRef = useRef(false);
    const alarmIntervalRef = useRef(null);
    const spokenAlertsRef = useRef(new Set());
    const ttsQueueRef = useRef([]);
    const isSpeakingRef = useRef(false);

    /* ── Sound helpers (same as AlertsPanel) ──────────────────────── */
    function startBuzz() {
        if (isBuzzingRef.current) return;
        try {
            const AudioCtx = window.AudioContext || window.webkitAudioContext;
            if (!AudioCtx) return;
            const ctx = new AudioCtx();
            const gain = ctx.createGain();
            gain.connect(ctx.destination);
            const osc = ctx.createOscillator();
            osc.type = 'sine';
            osc.frequency.setValueAtTime(880, ctx.currentTime);
            osc.connect(gain);
            osc.start();
            gain.gain.setValueAtTime(0, ctx.currentTime);
            audioContextRef.current = ctx;
            oscillator1Ref.current = osc;
            gainRef.current = gain;
            isBuzzingRef.current = true;

            const playBeepPattern = () => {
                if (!audioContextRef.current) return;
                const c = audioContextRef.current;
                const now = c.currentTime;
                const g = gainRef.current;
                const dur = 0.15, gap = 0.15, vol = 0.3;
                g.gain.cancelScheduledValues(now);
                g.gain.setValueAtTime(0, now);
                g.gain.linearRampToValueAtTime(vol, now + 0.02);
                g.gain.linearRampToValueAtTime(0, now + dur);
                const s2 = now + dur + gap;
                g.gain.setValueAtTime(0, s2);
                g.gain.linearRampToValueAtTime(vol, s2 + 0.02);
                g.gain.linearRampToValueAtTime(0, s2 + dur);
                const s3 = s2 + dur + gap;
                g.gain.setValueAtTime(0, s3);
                g.gain.linearRampToValueAtTime(vol, s3 + 0.02);
                g.gain.linearRampToValueAtTime(0, s3 + dur);
            };
            playBeepPattern();
            alarmIntervalRef.current = setInterval(playBeepPattern, 3000);
        } catch (e) { console.error('Buzz start error', e); }
    }

    function stopBuzz() {
        try {
            if (alarmIntervalRef.current) { clearInterval(alarmIntervalRef.current); alarmIntervalRef.current = null; }
            if (isBuzzingRef.current) {
                const ctx = audioContextRef.current, osc = oscillator1Ref.current, g = gainRef.current;
                if (g && ctx) { g.gain.cancelScheduledValues(ctx.currentTime); g.gain.setValueAtTime(0, ctx.currentTime); }
                if (osc) osc.stop(ctx ? ctx.currentTime + 0.05 : undefined);
                if (ctx && typeof ctx.close === 'function') setTimeout(() => ctx.close().catch(() => { }), 100);
            }
        } catch (e) { console.error('Buzz stop error', e); } finally {
            audioContextRef.current = null; oscillator1Ref.current = null; gainRef.current = null; isBuzzingRef.current = false;
        }
    }

    /* ── Fetch alerts ─────────────────────────────────────────────── */
    useEffect(() => {
        fetchAlerts();
        const id = setInterval(fetchAlerts, 30000);
        return () => clearInterval(id);
    }, [fetchAlerts]);

    /* ── Alert mapping helpers (same logic as AlertsPanel) ────────── */
    function getIngredientFromAlert(alert) {
        if (alert.payload) {
            let p = typeof alert.payload === 'string' ? (() => { try { return JSON.parse(alert.payload); } catch { return null; } })() : alert.payload;
            if (p?.ingredient) return p.ingredient;
        }
        const msg = alert.message || '';
        return ['cup_stations', 'milk', 'cup', 'beans', 'syrup', 'coffee'].find(i => msg.toLowerCase().includes(i));
    }

    function getSeverityFromAlert(alert) {
        if (alert.payload) {
            let p = typeof alert.payload === 'string' ? (() => { try { return JSON.parse(alert.payload); } catch { return null; } })() : alert.payload;
            if (p) return p.severity || alert.severity || 'low';
        }
        return alert.severity || 'low';
    }

    function getDefaultMessage(type) {
        const m = { ingredient_threshold: 'Below threshold', order_halted: 'Order halted', emergency_stop: 'E-stop activated', hardware: 'Hardware issue' };
        return m[type] || 'Needs attention';
    }

    function getAlertTitle(alert) {
        if (alert.alert_type === 'ingredient_threshold') {
            const ing = getIngredientFromAlert(alert);
            if (ing === 'cup_stations') return 'Cup Stations';
            const d = ing ? getIngredientData(ing) : null;
            const sev = getSeverityFromAlert(alert);
            const sevTxt = sev === 'empty' ? 'Out of Stock' : 'Low Stock';
            return d?.name ? `${d.name} - ${sevTxt}` : `Low ${ing ? ing.charAt(0).toUpperCase() + ing.slice(1) : 'Ingredient'}`;
        }
        if (alert.alert_type === 'order_halted') return 'Order Halted';
        if (alert.alert_type === 'emergency_stop') return 'E-Stop';
        if (alert.alert_type === 'hardware') return 'Hardware Issue';
        return alert.message || 'Alert';
    }

    function getAlertMessage(alert) {
        if (alert.alert_type === 'ingredient_threshold') return getSeverityMessage(getSeverityFromAlert(alert));
        if (alert.alert_type === 'order_halted' && alert.payload) {
            let p = typeof alert.payload === 'string' ? (() => { try { return JSON.parse(alert.payload); } catch { return null; } })() : alert.payload;
            if (p?.validation_function) return getValidationMessage(p.validation_function, 'Order halted');
        }
        return alert.message || getDefaultMessage(alert.alert_type);
    }

    function getValidationDataFromAlert(alert) {
        if (alert.alert_type === 'order_halted' && alert.payload) {
            let p = typeof alert.payload === 'string' ? (() => { try { return JSON.parse(alert.payload); } catch { return null; } })() : alert.payload;
            if (p?.validation_function) return getValidationData(p.validation_function);
        }
        return { icon: null, color: 'text-gray-600' };
    }

    function mapType(alertType, severity) {
        if (severity === 'critical') return 'error';
        if (alertType === 'ingredient_threshold') return 'warning';
        if (['emergency_stop', 'order_halted', 'hardware'].includes(alertType)) return 'error';
        return 'info';
    }

    const mappedAlerts = alerts.map(a => {
        const ingredient = getIngredientFromAlert(a);
        const vd = getValidationDataFromAlert(a);
        const id = a.alert_type === 'ingredient_threshold' && ingredient ? getIngredientData(ingredient) : null;
        return {
            id: a.id, type: mapType(a.alert_type, a.severity),
            title: getAlertTitle(a), message: getAlertMessage(a),
            validationIcon: vd.icon, validationColor: vd.color,
            ingredientIcon: id?.icon, ingredientColor: id?.color,
            timestamp: a.created_at ? new Date(a.created_at) : new Date(),
            severity: a.severity || 'medium', rawAlert: a
        };
    });

    const unacknowledgedAlerts = mappedAlerts;

    /* ── Buzzer lifecycle ─────────────────────────────────────────── */
    useEffect(() => {
        if (!isLoading && unacknowledgedAlerts.length > 0) startBuzz(); else stopBuzz();
        return () => stopBuzz();
    }, [unacknowledgedAlerts.length, isLoading]);

    /* ── TTS ───────────────────────────────────────────────────────── */
    function processTtsQueue() {
        if (isSpeakingRef.current || !window?.speechSynthesis) return;
        const q = ttsQueueRef.current;
        if (!q.length) return;
        try {
            const utter = new SpeechSynthesisUtterance(q.shift());
            utter.rate = 1; utter.pitch = 1; utter.volume = 1;
            const voices = window.speechSynthesis.getVoices?.() || [];
            const en = voices.find(v => v.lang?.toLowerCase().startsWith('en'));
            if (en) utter.voice = en;
            isSpeakingRef.current = true;
            utter.onend = () => { isSpeakingRef.current = false; processTtsQueue(); };
            utter.onerror = () => { isSpeakingRef.current = false; processTtsQueue(); };
            window.speechSynthesis.speak(utter);
        } catch { isSpeakingRef.current = false; }
    }

    useEffect(() => {
        if (!isLoading && unacknowledgedAlerts.length > 0) {
            unacknowledgedAlerts.filter(a => !spokenAlertsRef.current.has(a.id)).forEach(a => {
                spokenAlertsRef.current.add(a.id);
                const txt = (a.message || '').replace(/\s+/g, ' ').trim();
                if (txt) ttsQueueRef.current.push(txt);
            });
            processTtsQueue();
        } else if (unacknowledgedAlerts.length === 0) {
            spokenAlertsRef.current = new Set();
            try { window?.speechSynthesis?.cancel(); } catch { }
        }
        return () => { try { window?.speechSynthesis?.cancel(); } catch { } };
    }, [unacknowledgedAlerts.length, isLoading]);

    /* ── Handlers ─────────────────────────────────────────────────── */
    const handleAcknowledge = async (id) => {
        if (acknowledging.has(id)) return;
        setAcknowledging(p => new Set(p).add(id));
        try { await acknowledgeAlert(id); } catch (e) { console.error(e); }
        finally { setAcknowledging(p => { const s = new Set(p); s.delete(id); return s; }); }
    };

    const handleAcknowledgeAll = async () => {
        if (!unacknowledgedAlerts.length) return;
        await Promise.all(unacknowledgedAlerts.map(a => handleAcknowledge(a.id)));
    };

    const handleNavigate = () => { navigateToTab('alerts'); window.location.hash = '#/alerts'; };

    const getTimeAgo = (ts) => {
        const diff = Date.now() - ts.getTime();
        const m = Math.floor(diff / 60000), h = Math.floor(diff / 3600000);
        return h > 0 ? `${h}h` : m > 0 ? `${m}m` : 'now';
    };

    const dotColor = { error: 'bg-red-500', warning: 'bg-amber-500', info: 'bg-blue-500' };

    /* ── Render ────────────────────────────────────────────────────── */
    return (
        <div className="bg-white rounded-xl flex flex-col h-full overflow-hidden" style={{ border: '1px solid #e5e7eb' }}>
            {/* Blink CSS */}
            <style>{`
        @keyframes alertBlink { 0%,100%{background-color:transparent} 50%{background-color:rgba(226,92,83,.35)} }
        .compact-alert-blink { animation: alertBlink 2s ease-in-out infinite; }
        .compact-alert-blink-critical { animation: alertBlink 1.4s ease-in-out infinite; }
      `}</style>

            {/* Header */}
            <div className="flex items-center justify-between px-3 py-2 border-b border-gray-100 flex-shrink-0">
                <div className="flex items-center gap-2">
                    <h2 className="text-sm font-bold text-gray-900">{t('active')}</h2>
                    {unacknowledgedAlerts.length > 0 && (
                        <span className="text-[10px] font-bold px-1.5 py-0.5 rounded-full bg-red-100 text-red-700">
                            {unacknowledgedAlerts.length}
                        </span>
                    )}
                </div>

                <div className="flex items-center gap-1.5">
                    {errors.alerts && (
                        <button
                            onClick={() => { clearError('alerts'); fetchAlerts(); }}
                            className="text-[10px] font-bold px-2 py-1 rounded-lg bg-red-50 text-red-600 hover:bg-red-100 transition-colors"
                            style={{ border: '1px solid #fca5a5' }}
                        >
                            {t('retry')}
                        </button>
                    )}
                    {unacknowledgedAlerts.length > 1 && (
                        <button
                            onClick={handleAcknowledgeAll}
                            disabled={isLoading || acknowledging.size > 0}
                            className="text-[10px] font-bold px-2 py-1 rounded-lg text-white disabled:opacity-40"
                            style={{ backgroundColor: '#233746', border: 'none' }}
                        >
                            Ack All
                        </button>
                    )}
                    <button
                        onClick={handleNavigate}
                        className="w-7 h-7 flex items-center justify-center rounded-lg hover:bg-gray-100 transition-colors"
                        style={{ border: '1px solid #d1d5db', padding: 0, minWidth: '28px', minHeight: '28px' }}
                        title="View all alerts"
                    >
                        <svg className="w-3.5 h-3.5 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2">
                            <path strokeLinecap="round" strokeLinejoin="round" d="M13 7l5 5m0 0l-5 5m5-5H6" />
                        </svg>
                    </button>
                </div>
            </div>

            {/* Error banner */}
            {errors.alerts && (
                <div className="px-2 py-1 bg-yellow-50 border-b border-yellow-200">
                    <p className="text-[10px] text-yellow-800 truncate">
                        {errors.alerts === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.alerts}
                    </p>
                </div>
            )}

            {/* Alerts list */}
            <div className="flex-1 overflow-y-auto" style={{ scrollbarWidth: 'thin', scrollbarColor: '#d1d5db transparent' }}>
                {isLoading && unacknowledgedAlerts.length === 0 ? (
                    <div className="flex flex-col items-center justify-center h-full gap-2 p-4">
                        <svg className="animate-spin w-5 h-5 text-blue-500" fill="none" viewBox="0 0 24 24">
                            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
                        </svg>
                        <span className="text-xs text-gray-400">{t('loadingAlerts')}</span>
                    </div>
                ) : unacknowledgedAlerts.length === 0 ? (
                    <div className="flex flex-col items-center justify-center h-full gap-1.5 p-4">
                        <svg className="w-8 h-8 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2">
                            <path strokeLinecap="round" strokeLinejoin="round" d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                        </svg>
                        <span className="text-xs font-medium text-gray-500">{t('allClear')}</span>
                        <span className="text-[10px] text-gray-400">{t('noActiveAlerts')}</span>
                    </div>
                ) : (
                    <div className="p-2 space-y-1.5">
                        {unacknowledgedAlerts.map(alert => (
                            <div
                                key={alert.id}
                                className={`rounded-xl p-2.5 transition-colors ${alert.severity === 'critical' ? 'compact-alert-blink-critical' : 'compact-alert-blink'
                                    }`}
                                style={{ border: '1px solid #e5e7eb' }}
                            >
                                {/* Row: icon + title + time */}
                                <div className="flex items-start gap-2">
                                    {/* Icon */}
                                    <div className="flex-shrink-0 mt-0.5">
                                        {(alert.validationIcon || alert.ingredientIcon) ? (
                                            <div className={`${alert.validationColor || alert.ingredientColor || 'text-gray-600'} [&_svg]:w-5 [&_svg]:h-5`}>
                                                {alert.validationIcon || alert.ingredientIcon}
                                            </div>
                                        ) : (
                                            <div className={`w-2.5 h-2.5 rounded-full mt-1 ${dotColor[alert.type] || 'bg-gray-400'}`} />
                                        )}
                                    </div>

                                    {/* Content */}
                                    <div className="flex-1 min-w-0">
                                        <div className="flex items-center gap-2">
                                            <span className="text-xs font-medium text-gray-900 leading-snug flex-1 min-w-0">{alert.title}</span>
                                            <span className="text-[11px] text-gray-400 flex-shrink-0">{getTimeAgo(alert.timestamp)}</span>
                                            <button
                                                onClick={(e) => { e.stopPropagation(); handleAcknowledge(alert.id); }}
                                                disabled={isLoading || acknowledging.has(alert.id) || !!errors.alerts}
                                                className="w-8 h-8 flex items-center justify-center rounded-lg hover:bg-green-100 active:scale-90 transition-all disabled:opacity-40 disabled:cursor-not-allowed flex-shrink-0"
                                                style={{ border: '2px solid #22c55e', backgroundColor: '#f0fdf4', padding: 0 }}
                                                title={t('acknowledge')}
                                            >
                                                {acknowledging.has(alert.id) ? (
                                                    <svg className="animate-spin w-4 h-4" style={{ color: '#16a34a' }} fill="none" viewBox="0 0 24 24">
                                                        <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                                                        <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" />
                                                    </svg>
                                                ) : (
                                                    <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" strokeWidth="3" style={{ stroke: '#16a34a' }}>
                                                        <path strokeLinecap="round" strokeLinejoin="round" d="M5 13l4 4L19 7" />
                                                    </svg>
                                                )}
                                            </button>
                                        </div>
                                        {alert.message && (
                                            <p className="text-[12px] text-black-400 leading-snug mt-1 pr-1">{alert.message}</p>
                                        )}
                                    </div>
                                </div>
                            </div>
                        ))}
                    </div>
                )}
            </div>
        </div>
    );
}
