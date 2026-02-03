/**
 * Translations Store
 * Manages per-page language translations with localStorage persistence.
 * Structure: { pages: { pageId: { key: { langCode: "text" } } }, languages: [{ code, name }], currentLocale: "en" }
 */

import { create } from 'zustand';
import { DEFAULT_TRANSLATIONS, DEFAULT_LANGUAGES } from '../constants/translations';

const STORAGE_KEY = 'barns_translations_v2';

function normalizeLanguages(languages) {
  if (!Array.isArray(languages) || languages.length === 0) {
    return [...DEFAULT_LANGUAGES];
  }
  return languages.map((item) => {
    if (typeof item === 'string') return { code: item, name: item };
    return { code: (item.code || '').trim().toLowerCase(), name: (item.name || item.code || '').trim() || item.code };
  }).filter((item) => item.code);
}

function getDefaultTranslation(pageId, key, langCode) {
  const page = DEFAULT_TRANSLATIONS[pageId];
  if (!page) return null;
  const keyData = page[key];
  if (!keyData) return null;
  return keyData[langCode] ?? null;
}

const loadFromStorage = () => {
  try {
    if (typeof window === 'undefined') {
      return {
        pages: JSON.parse(JSON.stringify(DEFAULT_TRANSLATIONS)),
        languages: [...DEFAULT_LANGUAGES],
        currentLocale: 'en'
      };
    }
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      let languages = normalizeLanguages(parsed.languages);
      if (languages.length === 0) languages = [...DEFAULT_LANGUAGES];
      const codes = languages.map((l) => l.code);
      if (!codes.includes('ar')) languages = [...languages, { code: 'ar', name: 'Arabic' }];
      return {
        pages: parsed.pages || {},
        languages,
        currentLocale: typeof parsed.currentLocale === 'string' ? parsed.currentLocale : 'en'
      };
    }
    const legacy = window.localStorage.getItem('barns_translations_v1');
    if (legacy) {
      const parsed = JSON.parse(legacy);
      const langList = Array.isArray(parsed.languages) ? parsed.languages : ['en'];
      const languages = langList.map((c) => (typeof c === 'string' ? { code: c, name: c } : { code: c.code || c, name: c.name || c.code || c }));
      const hasAr = languages.some((l) => (l.code || l) === 'ar');
      if (!hasAr) languages.push({ code: 'ar', name: 'Arabic' });
      return {
        pages: parsed.pages || {},
        languages: languages.length ? languages : [...DEFAULT_LANGUAGES],
        currentLocale: typeof parsed.currentLocale === 'string' ? parsed.currentLocale : 'en'
      };
    }
    return {
      pages: JSON.parse(JSON.stringify(DEFAULT_TRANSLATIONS)),
      languages: [...DEFAULT_LANGUAGES],
      currentLocale: 'en'
    };
  } catch {
    return {
      pages: JSON.parse(JSON.stringify(DEFAULT_TRANSLATIONS)),
      languages: [...DEFAULT_LANGUAGES],
      currentLocale: 'en'
    };
  }
};

const saveToStorage = (state) => {
  try {
    if (typeof window === 'undefined') return;
    const payload = {
      pages: state.pages,
      languages: state.languages,
      currentLocale: state.currentLocale
    };
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(payload));
  } catch {}
};

function getLangCodes(languages) {
  return (languages || []).map((l) => (typeof l === 'string' ? l : l.code));
}

export const useTranslationsStore = create((set, get) => ({
  ...loadFromStorage(),

  addLanguage: (code, name) => {
    const c = (code || '').trim().toLowerCase();
    const n = (name || '').trim() || c;
    if (!c) return;
    set((state) => {
      const codes = getLangCodes(state.languages);
      if (codes.includes(c)) return state;
      const languages = [...state.languages, { code: c, name: n }];
      const pages = { ...state.pages };
      Object.keys(pages).forEach((pageId) => {
        const keys = pages[pageId] || {};
        Object.keys(keys).forEach((key) => {
          if (!keys[key][c]) keys[key] = { ...keys[key], [c]: '' };
        });
      });
      const next = { ...state, languages, pages };
      saveToStorage(next);
      return next;
    });
  },

  updateLanguageName: (code, name) => {
    const c = (code || '').trim().toLowerCase();
    const n = (name || '').trim();
    if (!c) return;
    set((state) => {
      const languages = state.languages.map((l) => (l.code === c ? { ...l, name: n || l.name } : l));
      const next = { ...state, languages };
      saveToStorage(next);
      return next;
    });
  },

  updateLanguage: (oldCode, newCode, newName) => {
    const oldC = (oldCode || '').trim().toLowerCase();
    const newC = (newCode || '').trim().toLowerCase();
    const newN = (newName || '').trim() || newC;
    if (!oldC) return;
    set((state) => {
      const codes = getLangCodes(state.languages);
      if (newC && newC !== oldC && codes.includes(newC)) return state;
      const languages = state.languages.map((l) =>
        l.code === oldC ? { code: newC || oldC, name: newN || l.name } : l
      );
      const pages = { ...state.pages };
      if (newC && newC !== oldC) {
        Object.keys(pages).forEach((pageId) => {
          const keys = pages[pageId] || {};
          Object.keys(keys).forEach((key) => {
            const byLang = keys[key] || {};
            if (byLang[oldC] !== undefined) {
              keys[key] = { ...byLang, [newC]: byLang[oldC] };
              delete keys[key][oldC];
            }
          });
        });
      }
      let currentLocale = state.currentLocale;
      if (currentLocale === oldC) currentLocale = newC || oldC;
      const next = { ...state, languages, pages, currentLocale };
      saveToStorage(next);
      return next;
    });
  },

  removeLanguage: (langCode) => {
    const code = (langCode || '').trim().toLowerCase();
    if (!code || code === 'en') return;
    set((state) => {
      const languages = state.languages.filter((l) => (typeof l === 'string' ? l : l.code) !== code);
      const pages = { ...state.pages };
      Object.keys(pages).forEach((pageId) => {
        const keys = pages[pageId] || {};
        const nextKeys = {};
        Object.entries(keys).forEach(([key, byLang]) => {
          const rest = { ...byLang };
          delete rest[code];
          if (Object.keys(rest).length) nextKeys[key] = rest;
        });
        pages[pageId] = Object.keys(nextKeys).length ? nextKeys : undefined;
      });
      const next = { ...state, languages, pages };
      if (state.currentLocale === code) next.currentLocale = 'en';
      saveToStorage(next);
      return next;
    });
  },

  setCurrentLocale: (locale) => {
    const nextLocale = (locale || 'en').trim().toLowerCase();
    set((state) => {
      const next = { ...state, currentLocale: nextLocale };
      saveToStorage(next);
      return next;
    });
  },

  setTranslation: (pageId, key, langCode, value) => {
    const page = (pageId || '').trim();
    const k = (key || '').trim();
    const lang = (langCode || '').trim().toLowerCase();
    if (!page || !k || !lang) return;
    set((state) => {
      const pages = { ...state.pages };
      const pageData = { ...(pages[page] || {}) };
      const keyData = { ...(pageData[k] || {}) };
      keyData[lang] = value == null ? '' : String(value);
      pageData[k] = keyData;
      pages[page] = pageData;
      const next = { ...state, pages };
      saveToStorage(next);
      return next;
    });
  },

  addPageKey: (pageId, key) => {
    const page = (pageId || '').trim();
    const k = (key || '').trim();
    if (!page || !k) return;
    set((state) => {
      const pages = { ...state.pages };
      const pageData = { ...(pages[page] || {}) };
      if (pageData[k]) return state;
      pageData[k] = {};
      getLangCodes(state.languages).forEach((lang) => {
        pageData[k][lang] = '';
      });
      pages[page] = pageData;
      const next = { ...state, pages };
      saveToStorage(next);
      return next;
    });
  },

  removePageKey: (pageId, key) => {
    const page = (pageId || '').trim();
    const k = (key || '').trim();
    if (!page || !k) return;
    set((state) => {
      const pages = { ...state.pages };
      const pageData = { ...(pages[page] || {}) };
      delete pageData[k];
      if (Object.keys(pageData).length === 0) delete pages[page];
      else pages[page] = pageData;
      const next = { ...state, pages };
      saveToStorage(next);
      return next;
    });
  },

  getTranslation: (pageId, key, langCode) => {
    const state = get();
    const lang = (langCode || state.currentLocale || 'en').trim().toLowerCase();
    const pageData = state.pages[pageId];
    const keyData = pageData?.[key];
    const stored = keyData?.[lang];
    if (stored != null && stored !== '') return stored;
    if (keyData?.en != null && keyData.en !== '') return keyData.en;
    const defaultText = getDefaultTranslation(pageId, key, lang);
    if (defaultText != null && defaultText !== '') return defaultText;
    const defaultEn = getDefaultTranslation(pageId, key, 'en');
    if (defaultEn != null && defaultEn !== '') return defaultEn;
    return null;
  },

  getPageTranslations: (pageId) => {
    return get().pages[pageId] || {};
  },

  getAllKeysByPage: () => {
    const state = get();
    return state.pages || {};
  }
}));

/**
 * Hook for use in page components. Returns t(key) that resolves translation for current locale.
 */
export function useTranslation(pageId) {
  const locale = useTranslationsStore((s) => s.currentLocale);
  const setLocale = useTranslationsStore((s) => s.setCurrentLocale);
  const getTranslation = useTranslationsStore((s) => s.getTranslation);
  const t = (key) => {
    const value = getTranslation(pageId, key, locale);
    return value != null ? value : key;
  };
  return { t, locale, setLocale };
}
