/**
 * Page identifiers for the Translations settings panel.
 * Translation keys are defined in constants/translations/keys.js.
 */

import { TRANSLATION_KEYS } from './translations/keys';

export const TRANSLATION_PAGES = [
  { id: 'dashboard', name: 'Dashboard' },
  { id: 'alerts', name: 'Alerts' },
  { id: 'inventory', name: 'Inventory' },
  { id: 'cameras', name: 'Cameras' },
  { id: 'settings', name: 'Settings' },
  { id: 'newOrder', name: 'New Order' }
];

export const DEFAULT_PAGE_KEYS = TRANSLATION_KEYS;
