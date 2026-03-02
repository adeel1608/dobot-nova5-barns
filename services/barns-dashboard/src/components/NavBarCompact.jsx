/**
 * NavBarCompact
 * Condensed navigation header optimized for POS/1024px-wide layouts.
 * Target height: ~48px. Logo is smaller, tabs are tighter.
 * QSS logo and profile are removed; language selector and alert bell are retained.
 */

import React, { useRef, useState, useEffect } from 'react';
import { useTranslationsStore, useTranslation } from '../store/translationsStore';
import barnsLogo from '../assets/barns.png';
import notification from '../assets/notification.png';

const NAV_TABS = [
  { key: 'dashboard', labelKey: 'navDashboard' },
  { key: 'alerts', labelKey: 'navNotifications' },
  { key: 'inventory', labelKey: 'navInventory' },
  { key: 'cameras', labelKey: 'navCameras' },
  { key: 'settings', labelKey: 'navSettings' },
];

export default function NavBarCompact({ activeTab, setActiveTab, alerts }) {
  const { t: tApp } = useTranslation('app');
  const { languages, currentLocale, setCurrentLocale } = useTranslationsStore();
  const [langDropdownOpen, setLangDropdownOpen] = useState(false);
  const langDropdownRef = useRef(null);

  const alertCount = alerts?.length ?? 0;

  useEffect(() => {
    const handleClickOutside = (e) => {
      if (langDropdownRef.current && !langDropdownRef.current.contains(e.target)) {
        setLangDropdownOpen(false);
      }
    };
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  return (
    <header className="bg-gray-100 border-b border-gray-200 z-10 flex-shrink-0 px-4 h-12 flex items-center">
      <div className="w-full flex items-center justify-between gap-4">

        {/* Left: compact logo */}
        <img
          src={barnsLogo}
          alt="BARNS"
          className="object-contain flex-shrink-0"
          style={{ height: 30 }}
        />

        {/* Center: nav tabs */}
        <nav className="flex items-center gap-0.5 flex-1 justify-center">
          {NAV_TABS.map(({ key, labelKey }) => (
            <button
              key={key}
              type="button"
              onClick={() => {
                setActiveTab(key);
                window.location.hash = `#/${key}`;
              }}
              style={{ boxShadow: 'none', height: '35px', minHeight: '30px', padding: '0 12px', display: 'flex', alignItems: 'center' }}
              className={`relative rounded-lg text-xs font-semibold transition-all duration-200 ${activeTab === key
                ? 'bg-green-800 text-white'
                : 'text-gray-500 hover:text-green-800 hover:bg-green-50'
                }`}
            >
              {tApp(labelKey)}
              {/* Alert badge on Notifications tab */}
              {key === 'alerts' && alertCount > 0 && (
                <span className="absolute -top-1 -right-1 barns-dark-bg text-white text-[8px] font-bold rounded-full w-3.5 h-3.5 flex items-center justify-center">
                  {alertCount > 9 ? '9+' : alertCount}
                </span>
              )}
            </button>
          ))}
        </nav>

        {/* Right: language selector */}
        <div className="flex items-center gap-2 flex-shrink-0">
          <div className="relative" ref={langDropdownRef}>
            <button
              type="button"
              onClick={() => setLangDropdownOpen((v) => !v)}
              style={{ padding: 0, height: '30px', width: '30px', minHeight: '30px', minWidth: '30px', display: 'flex', alignItems: 'center', justifyContent: 'center' }}
              className="rounded-lg text-gray-500 hover:bg-gray-200 transition-colors flex-shrink-0"
              title="Change language"
              aria-label="Change language"
            >
              <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
                  d="M21 12a9 9 0 01-9 9m9-9a9 9 0 00-9-9m9 9H3m9 9a9 9 0 01-9-9m9 9c1.657 0 3-4.03 3-9s-1.343-9-3-9m0 18c-1.657 0-3-4.03-3-9s1.343-9 3-9m-9 9a9 9 0 019-9" />
              </svg>
            </button>
            {langDropdownOpen && (
              <div className="absolute right-0 mt-1 w-40 bg-white rounded-lg shadow-lg border border-gray-200 py-1 z-50">
                {languages.map((lang) => (
                  <button
                    key={lang.code}
                    type="button"
                    onClick={() => {
                      setCurrentLocale(lang.code);
                      setLangDropdownOpen(false);
                    }}
                    className={`w-full text-left px-3 py-1.5 text-xs transition-colors ${currentLocale === lang.code
                      ? 'bg-green-50 text-green-800 font-semibold'
                      : 'text-gray-700 hover:bg-gray-50'
                      }`}
                  >
                    {lang.name} ({lang.code})
                  </button>
                ))}
              </div>
            )}
          </div>
        </div>

      </div>
    </header>
  );
}
