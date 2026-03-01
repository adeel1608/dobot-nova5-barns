/**
 * Display Store
 * Manages display/resolution mode preferences with localStorage persistence.
 * Modes: 'default' (native screen size) | 'pos' (1024x768 compact layout)
 */

import { create } from 'zustand';

const STORAGE_KEY = 'barns_display_settings';

const VALID_MODES = ['default', 'pos'];

function loadFromStorage() {
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      if (VALID_MODES.includes(parsed.resolutionMode)) {
        return { resolutionMode: parsed.resolutionMode };
      }
    }
  } catch {
    // Ignore parse errors and fall through to default
  }
  return { resolutionMode: 'default' };
}

function saveToStorage(state) {
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify({
      resolutionMode: state.resolutionMode,
    }));
  } catch {
    // Ignore write errors (e.g. private browsing quota)
  }
}

const savedState = loadFromStorage();

export const useDisplayStore = create((set) => ({
  resolutionMode: savedState.resolutionMode,

  setResolutionMode: (mode) => {
    if (!VALID_MODES.includes(mode)) return;
    set((state) => {
      const next = { ...state, resolutionMode: mode };
      saveToStorage(next);
      return { resolutionMode: mode };
    });
  },
}));

/** Convenience selector — returns true when POS (1024x768) mode is active. */
export function useIsPosMode() {
  return useDisplayStore((state) => state.resolutionMode === 'pos');
}
