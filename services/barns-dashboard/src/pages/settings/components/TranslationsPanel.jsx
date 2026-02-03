/**
 * Translations Panel
 * Table of languages with Translation, Edit, Export file, and Delete actions.
 * Add/Edit language modals: name and short name (code). Translation modal: all keys per page for that language.
 */

import React, { useState, useMemo } from 'react';
import { useTranslationsStore, useTranslation } from '../../../store/translationsStore';
import { TRANSLATION_PAGES, DEFAULT_PAGE_KEYS } from '../../../constants/translationPages';
import { DEFAULT_TRANSLATIONS, getNewLanguageFileContent } from '../../../constants/translations';

function Modal({ open, onClose, title, children }) {
  if (!open) return null;
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/50" onClick={onClose}>
      <div
        className="bg-white rounded-xl shadow-lg border border-gray-200 max-w-2xl w-full max-h-[90vh] overflow-hidden flex flex-col"
        onClick={(e) => e.stopPropagation()}
      >
        <div className="px-6 py-4 border-b border-gray-200 flex items-center justify-between">
          <h3 className="text-lg font-semibold text-gray-900">{title}</h3>
          <button
            type="button"
            onClick={onClose}
            className="p-1 rounded text-gray-500 hover:text-gray-700 hover:bg-gray-100"
            aria-label="Close"
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
        <div className="flex-1 overflow-y-auto p-6">{children}</div>
      </div>
    </div>
  );
}

export default function TranslationsPanel() {
  const {
    pages,
    languages,
    currentLocale,
    addLanguage,
    removeLanguage,
    setCurrentLocale,
    setTranslation,
    updateLanguage,
    getPageTranslations,
    getAllKeysByPage
  } = useTranslationsStore();
  const { t } = useTranslation('settings');

  const [addLanguageOpen, setAddLanguageOpen] = useState(false);
  const [addLanguageName, setAddLanguageName] = useState('');
  const [addLanguageCode, setAddLanguageCode] = useState('');
  const [editLanguageOpen, setEditLanguageOpen] = useState(false);
  const [editLanguage, setEditLanguage] = useState(null);
  const [editName, setEditName] = useState('');
  const [editCode, setEditCode] = useState('');
  const [translationModalLang, setTranslationModalLang] = useState(null);
  const [deleteConfirmLang, setDeleteConfirmLang] = useState(null);
  const [notification, setNotification] = useState(null);

  const showNotification = (message, type = 'success') => {
    setNotification({ message, type });
    setTimeout(() => setNotification(null), 3000);
  };

  const langCodes = useMemo(() => languages.map((l) => l.code), [languages]);

  const handleAddLanguageOpen = () => {
    setAddLanguageName('');
    setAddLanguageCode('');
    setAddLanguageOpen(true);
  };

  const handleAddLanguageSave = () => {
    const code = (addLanguageCode || '').trim().toLowerCase();
    const name = (addLanguageName || '').trim() || code;
    if (!code) {
      showNotification('Short name (code) is required', 'error');
      return;
    }
    if (langCodes.includes(code)) {
      showNotification('A language with this code already exists', 'error');
      return;
    }
    addLanguage(code, name);
    setAddLanguageOpen(false);
    showNotification(`Language "${name}" (${code}) added`);
  };

  const handleTranslationOpen = (lang) => {
    setTranslationModalLang(lang);
  };

  const handleTranslationClose = () => {
    setTranslationModalLang(null);
  };

  const handleDeleteClick = (lang) => {
    if (lang.code === 'en') return;
    setDeleteConfirmLang(lang);
  };

  const handleDeleteConfirm = () => {
    if (deleteConfirmLang) {
      removeLanguage(deleteConfirmLang.code);
      setDeleteConfirmLang(null);
      showNotification(`Language "${deleteConfirmLang.name}" removed`);
    }
  };

  const handleDeleteCancel = () => {
    setDeleteConfirmLang(null);
  };

  const handleEditOpen = (lang) => {
    setEditLanguage(lang);
    setEditName(lang.name);
    setEditCode(lang.code);
    setEditLanguageOpen(true);
  };

  const handleEditClose = () => {
    setEditLanguageOpen(false);
    setEditLanguage(null);
    setEditName('');
    setEditCode('');
  };

  const handleEditSave = () => {
    if (!editLanguage) return;
    const newCode = (editCode || '').trim().toLowerCase();
    const newName = (editName || '').trim() || newCode;
    if (!newCode) {
      showNotification('Code is required', 'error');
      return;
    }
    const otherCodes = langCodes.filter((c) => c !== editLanguage.code);
    if (otherCodes.includes(newCode)) {
      showNotification('A language with this code already exists', 'error');
      return;
    }
    updateLanguage(editLanguage.code, newCode, newName);
    handleEditClose();
    showNotification(`Language updated to "${newName}" (${newCode})`);
  };

  const handleExportLanguageFile = (lang) => {
    const getValue = (pageId, key) => {
      const fromStore = pages[pageId]?.[key]?.[lang.code];
      if (fromStore != null && fromStore !== '') return fromStore;
      return DEFAULT_TRANSLATIONS[pageId]?.[key]?.[lang.code] ?? '';
    };
    const content = getNewLanguageFileContent(lang.code, getValue);
    const blob = new Blob([content], { type: 'text/javascript' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${lang.code}.js`;
    a.click();
    URL.revokeObjectURL(url);
    showNotification(`Exported ${lang.name} (${lang.code}.js)`);
  };

  const allKeysByPage = useMemo(() => {
    const storePages = getAllKeysByPage();
    const result = {};
    TRANSLATION_PAGES.forEach((page) => {
      const defaultKeys = DEFAULT_PAGE_KEYS[page.id] || [];
      const storeKeys = Object.keys(storePages[page.id] || {});
      result[page.id] = [...new Set([...defaultKeys, ...storeKeys])].sort();
    });
    return result;
  }, [pages, getAllKeysByPage]);

  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-200 p-6">
      <div className="flex items-center justify-between mb-6">
        <div>
          <h2 className="text-lg font-semibold text-gray-900">{t('transTitle')}</h2>
          <p className="text-sm text-gray-600 mt-1">
            {t('transSubtitle')}
          </p>
        </div>
        <div className="flex items-center gap-3">
          <select
            value={currentLocale}
            onChange={(e) => setCurrentLocale(e.target.value)}
            className="rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            title={t('displayLanguage')}
          >
            {languages.map((lang) => (
              <option key={lang.code} value={lang.code}>
                {lang.name} ({lang.code})
              </option>
            ))}
          </select>
          <button
            type="button"
            onClick={handleAddLanguageOpen}
            className="px-4 py-2 bg-blue-600 text-white text-sm font-medium rounded-lg hover:bg-blue-700 focus:ring-2 focus:ring-blue-500"
          >
            {t('addLanguage')}
          </button>
        </div>
      </div>

      {notification && (
        <div
          className={`mb-4 px-4 py-2 rounded-lg text-sm ${
            notification.type === 'error' ? 'bg-red-50 text-red-700' : 'bg-green-50 text-green-700'
          }`}
        >
          {notification.message}
        </div>
      )}

      <div className="border border-gray-200 rounded-lg overflow-hidden">
        <table className="w-full text-sm">
          <thead className="bg-gray-50 border-b border-gray-200">
            <tr>
              <th className="text-left py-3 px-4 font-medium text-gray-700">{t('languageName')}</th>
              <th className="text-left py-3 px-4 font-medium text-gray-700">{t('code')}</th>
              <th className="text-right py-3 px-4 font-medium text-gray-700">{t('actions')}</th>
            </tr>
          </thead>
          <tbody>
            {languages.map((lang) => (
              <tr key={lang.code} className="border-b border-gray-100 last:border-0 hover:bg-gray-50/50">
                <td className="py-3 px-4 text-gray-900">{lang.name}</td>
                <td className="py-3 px-4 font-mono text-gray-600">{lang.code}</td>
                <td className="py-3 px-4 text-right">
                  <div className="flex items-center justify-end gap-2 flex-wrap">
                    <button
                      type="button"
                      onClick={() => handleTranslationOpen(lang)}
                      className="px-3 py-1.5 text-sm font-medium text-blue-600 hover:bg-blue-50 rounded-lg"
                    >
                      {t('translation')}
                    </button>
                    <button
                      type="button"
                      onClick={() => handleEditOpen(lang)}
                      className="px-3 py-1.5 text-sm font-medium text-gray-700 hover:bg-gray-100 rounded-lg border border-gray-300"
                    >
                      {t('edit')}
                    </button>
                    <button
                      type="button"
                      onClick={() => handleExportLanguageFile(lang)}
                      className="px-3 py-1.5 text-sm font-medium text-gray-700 hover:bg-gray-100 rounded-lg border border-gray-300"
                    >
                      {t('exportFile')}
                    </button>
                    {lang.code !== 'en' && (
                      <button
                        type="button"
                        onClick={() => handleDeleteClick(lang)}
                        className="px-3 py-1.5 text-sm font-medium text-red-600 hover:bg-red-50 rounded-lg"
                      >
                        {t('delete')}
                      </button>
                    )}
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {languages.length === 0 && (
        <p className="text-sm text-gray-500 py-6 text-center">{t('noLanguagesYet')}</p>
      )}

      {/* Add language modal */}
      <Modal open={addLanguageOpen} onClose={() => setAddLanguageOpen(false)} title={t('addLanguage')}>
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">{t('name')}</label>
            <input
              type="text"
              value={addLanguageName}
              onChange={(e) => setAddLanguageName(e.target.value)}
              placeholder="e.g. Arabic, French"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">{t('shortName')}</label>
            <input
              type="text"
              value={addLanguageCode}
              onChange={(e) => setAddLanguageCode(e.target.value)}
              placeholder="e.g. ar, fr, en"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
            <p className="mt-1 text-xs text-gray-500">{t('addKeyHint')}</p>
          </div>
          <div className="flex justify-end gap-2 pt-2">
            <button
              type="button"
              onClick={() => setAddLanguageOpen(false)}
              className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 rounded-lg hover:bg-gray-200"
            >
              {t('cancel')}
            </button>
            <button
              type="button"
              onClick={handleAddLanguageSave}
              className="px-4 py-2 text-sm font-medium text-white bg-blue-600 rounded-lg hover:bg-blue-700"
            >
              {t('save')}
            </button>
          </div>
        </div>
      </Modal>

      {/* Edit language modal */}
      <Modal open={editLanguageOpen} onClose={handleEditClose} title={t('editLanguage')}>
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">{t('name')}</label>
            <input
              type="text"
              value={editName}
              onChange={(e) => setEditName(e.target.value)}
              placeholder="e.g. Arabic, French"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">{t('shortName')}</label>
            <input
              type="text"
              value={editCode}
              onChange={(e) => setEditCode(e.target.value)}
              placeholder="e.g. ar, fr, en"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            />
            <p className="mt-1 text-xs text-gray-500">{t('editCodeHint')}</p>
          </div>
          <div className="flex justify-end gap-2 pt-2">
            <button
              type="button"
              onClick={handleEditClose}
              className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 rounded-lg hover:bg-gray-200"
            >
              {t('cancel')}
            </button>
            <button
              type="button"
              onClick={handleEditSave}
              className="px-4 py-2 text-sm font-medium text-white bg-blue-600 rounded-lg hover:bg-blue-700"
            >
              {t('save')}
            </button>
          </div>
        </div>
      </Modal>

      {/* Translation modal: all keys for selected language */}
      <Modal
        open={!!translationModalLang}
        onClose={handleTranslationClose}
        title={translationModalLang ? `Translations for ${translationModalLang.name} (${translationModalLang.code})` : ''}
      >
        {translationModalLang && (
          <TranslationKeysForm
            langCode={translationModalLang.code}
            allKeysByPage={allKeysByPage}
            pages={pages}
            defaultTranslations={DEFAULT_TRANSLATIONS}
            setTranslation={setTranslation}
            onClose={handleTranslationClose}
            closeLabel={t('close')}
          />
        )}
      </Modal>

      {/* Delete confirmation */}
      {deleteConfirmLang && (
        <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black/50" onClick={handleDeleteCancel}>
          <div
            className="bg-white rounded-xl shadow-lg border border-gray-200 p-6 max-w-sm w-full"
            onClick={(e) => e.stopPropagation()}
          >
            <h3 className="text-lg font-semibold text-gray-900 mb-2">{t('deleteLanguage')}</h3>
            <p className="text-sm text-gray-600 mb-6">
              {t('removeConfirm').replace('{name}', deleteConfirmLang.name).replace('{code}', deleteConfirmLang.code)}
            </p>
            <div className="flex justify-end gap-2">
              <button
                type="button"
                onClick={handleDeleteCancel}
                className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 rounded-lg hover:bg-gray-200"
              >
                {t('cancel')}
              </button>
              <button
                type="button"
                onClick={handleDeleteConfirm}
                className="px-4 py-2 text-sm font-medium text-white bg-red-600 rounded-lg hover:bg-red-700"
              >
                {t('delete')}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

function TranslationKeysForm({ langCode, allKeysByPage, pages, defaultTranslations, setTranslation, onClose, closeLabel }) {
  const getValue = (pageId, key) => {
    const pageData = pages[pageId] || {};
    const keyData = pageData[key] || {};
    const stored = keyData[langCode];
    if (stored != null && stored !== '') return stored;
    const defaultPage = defaultTranslations?.[pageId];
    const defaultKey = defaultPage?.[key];
    return defaultKey?.[langCode] ?? '';
  };

  const hasAnyKeys = Object.values(allKeysByPage).some((keys) => keys.length > 0);

  if (!hasAnyKeys) {
    return (
      <div className="py-4">
        <p className="text-sm text-gray-500">No translation keys defined yet. Keys are added when pages use the translation system.</p>
        <div className="mt-4 flex justify-end">
            <button
              type="button"
              onClick={onClose}
              className="px-4 py-2 text-sm font-medium text-gray-700 bg-gray-100 rounded-lg hover:bg-gray-200"
            >
              {closeLabel || 'Close'}
            </button>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="max-h-96 overflow-y-auto space-y-6">
        {TRANSLATION_PAGES.map((page) => {
          const keys = allKeysByPage[page.id] || [];
          if (keys.length === 0) return null;
          return (
            <div key={page.id}>
              <h4 className="text-sm font-medium text-gray-700 mb-2">{page.name}</h4>
              <div className="space-y-2">
                {keys.map((key) => (
                  <div key={key} className="flex items-center gap-3">
                    <label className="w-40 shrink-0 text-sm text-gray-600 font-mono">{key}</label>
                    <input
                      type="text"
                      value={getValue(page.id, key)}
                      onChange={(e) => setTranslation(page.id, key, langCode, e.target.value)}
                      placeholder={`${key} (${langCode})`}
                      className="flex-1 rounded-lg border border-gray-300 px-3 py-2 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
                    />
                  </div>
                ))}
              </div>
            </div>
          );
        })}
      </div>
      <div className="flex justify-end pt-2 border-t border-gray-200">
        <button
          type="button"
          onClick={onClose}
          className="px-4 py-2 text-sm font-medium text-white bg-blue-600 rounded-lg hover:bg-blue-700"
        >
          {closeLabel || 'Close'}
        </button>
      </div>
    </div>
  );
}
