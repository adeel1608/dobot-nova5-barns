# Translations

One file per language (`en.js`, `ar.js`, etc.) and one file for all keywords (`keys.js`).

## Adding a new language (e.g. French)

1. In the dashboard: Settings > Translations > Add language (Name: French, Code: fr).
2. Click **Translation** for French, fill in the values, then click **Export file** to download `fr.js`.
3. Save the downloaded file as `fr.js` in this folder (`src/constants/translations/`).
4. In `index.js`:
   - Add: `import fr from './fr';`
   - Add `fr` to `LANG_MODULES`: `export const LANG_MODULES = { en, ar, fr };`
   - Add to `DEFAULT_LANGUAGES`: `{ code: 'fr', name: 'French' }`

## Adding new keys

Edit `keys.js`: add the key to the array for the page. Then add the same key and value to each language file (`en.js`, `ar.js`, etc.).
