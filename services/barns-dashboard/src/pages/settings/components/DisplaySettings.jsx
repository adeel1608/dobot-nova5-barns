/**
 * Display Settings Panel
 * Allows users to select the resolution mode for the dashboard.
 * Choices persist immediately to localStorage via displayStore.
 */

import React from 'react';
import { useDisplayStore } from '../../../store/displayStore';
import { useTranslation } from '../../../store/translationsStore';

function MonitorIcon({ className }) {
  return (
    <svg className={className} fill="none" stroke="currentColor" viewBox="0 0 24 24">
      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5}
        d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
    </svg>
  );
}

function CheckIcon({ className }) {
  return (
    <svg className={className} fill="none" stroke="currentColor" viewBox="0 0 24 24">
      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2.5} d="M5 13l4 4L19 7" />
    </svg>
  );
}

const MODES = [
  {
    id: 'default',
    titleKey: 'defaultMode',
    descKey: 'defaultModeDesc',
    resolution: null,
    icon: (
      <svg className="w-10 h-10" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5}
          d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
      </svg>
    ),
  },
  {
    id: 'pos',
    titleKey: 'posMode',
    descKey: 'posModeDesc',
    resolution: '1024 x 768',
    icon: (
      <svg className="w-10 h-10" fill="none" stroke="currentColor" viewBox="0 0 24 24">
        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={1.5}
          d="M12 18h.01M7 21h10a2 2 0 002-2V5a2 2 0 00-2-2H7a2 2 0 00-2 2v14a2 2 0 002 2z" />
      </svg>
    ),
  },
];

export default function DisplaySettings() {
  const { t } = useTranslation('settings');
  const { resolutionMode, setResolutionMode } = useDisplayStore();

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-6">
      {/* Section header */}
      <div className="flex items-center gap-3 mb-6">
        <div className="p-2 rounded-lg bg-blue-50 text-blue-600">
          <MonitorIcon className="w-5 h-5" />
        </div>
        <div>
          <h2 className="text-base font-semibold text-gray-900">{t('displaySettings')}</h2>
          <p className="text-sm text-gray-500 mt-0.5">{t('resolutionMode')}</p>
        </div>
      </div>

      {/* Mode cards */}
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
        {MODES.map((mode) => {
          const isActive = resolutionMode === mode.id;
          return (
            <button
              key={mode.id}
              type="button"
              onClick={() => setResolutionMode(mode.id)}
              className={`relative text-left p-5 rounded-xl border-2 transition-all duration-200 focus:outline-none ${
                isActive
                  ? 'border-blue-500 bg-blue-50'
                  : 'border-gray-200 bg-white hover:border-gray-300 hover:bg-gray-50'
              }`}
            >
              {/* Active checkmark badge */}
              {isActive && (
                <span className="absolute top-3 right-3 w-5 h-5 rounded-full bg-blue-500 flex items-center justify-center">
                  <CheckIcon className="w-3 h-3 text-white" />
                </span>
              )}

              {/* Icon */}
              <div className={`mb-3 ${isActive ? 'text-blue-600' : 'text-gray-400'}`}>
                {mode.icon}
              </div>

              {/* Title + resolution tag */}
              <div className="flex items-center gap-2 mb-1">
                <span className={`font-semibold text-sm ${isActive ? 'text-blue-700' : 'text-gray-800'}`}>
                  {t(mode.titleKey)}
                </span>
                {mode.resolution && (
                  <span className="text-[10px] font-bold bg-gray-200 text-gray-600 rounded px-1.5 py-0.5">
                    {mode.resolution}
                  </span>
                )}
              </div>

              {/* Description */}
              <p className="text-xs text-gray-500 leading-snug">{t(mode.descKey)}</p>

              {/* Active indicator text */}
              {isActive && (
                <p className="mt-2 text-[10px] font-semibold text-blue-600 uppercase tracking-wide">
                  {t('currentMode')}
                </p>
              )}
            </button>
          );
        })}
      </div>

      {/* Info note */}
      <p className="mt-5 text-xs text-gray-400">
        {resolutionMode === 'pos'
          ? t('posModeDesc')
          : t('defaultModeDesc')}
      </p>
    </div>
  );
}
