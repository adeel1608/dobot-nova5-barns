/**
 * Translations index.
 * Imports per-language files and merges into DEFAULT_TRANSLATIONS.
 * To add a new language: create e.g. fr.js with the same structure as en.js, then import and add to LANG_MODULES and DEFAULT_LANGUAGES.
 */

import { TRANSLATION_KEYS } from './keys';
import en from './en';
import ar from './ar';

export { TRANSLATION_KEYS };

export const LANG_MODULES = { en, ar };

export const DEFAULT_LANGUAGES = [
  { code: 'en', name: 'English' },
  { code: 'ar', name: 'Arabic' }
];

function mergeLangIntoDefault(acc, pageId, key, langCode, value) {
  if (!acc[pageId]) acc[pageId] = {};
  if (!acc[pageId][key]) acc[pageId][key] = {};
  acc[pageId][key][langCode] = value;
}

export const DEFAULT_TRANSLATIONS = (() => {
  const merged = {};
  Object.keys(LANG_MODULES).forEach((langCode) => {
    const mod = LANG_MODULES[langCode];
    Object.keys(TRANSLATION_KEYS).forEach((pageId) => {
      const keys = TRANSLATION_KEYS[pageId] || [];
      keys.forEach((key) => {
        const value = mod[pageId]?.[key] ?? '';
        mergeLangIntoDefault(merged, pageId, key, langCode, value);
      });
    });
  });
  return merged;
})();

/**
 * Generate JS file content for a new language with all keywords.
 * Use when a user adds a language in the UI: pass getValue(pageId, key) that returns the translation for that language; missing values become ''.
 * @param {string} langCode - e.g. 'fr'
 * @param {(pageId: string, key: string) => string} getValue - function that returns the translation for (pageId, key)
 * @returns {string} Content for e.g. fr.js
 */
export function getNewLanguageFileContent(langCode, getValue) {
  const lines = ["/**", ` * ${langCode} translations. Add this file to src/constants/translations/ and register in index.js.`, " */", "", "export default {"];
  Object.keys(TRANSLATION_KEYS).forEach((pageId) => {
    lines.push(`  ${pageId}: {`);
    const keys = TRANSLATION_KEYS[pageId] || [];
    keys.forEach((key) => {
      const raw = getValue(pageId, key) || '';
      const value = raw.replace(/\\/g, '\\\\').replace(/'/g, "\\'").replace(/\r/g, '\\r').replace(/\n/g, '\\n');
      lines.push(`    ${key}: '${value}',`);
    });
    lines.push("  },");
  });
  lines.push("};");
  return lines.join("\n");
}
